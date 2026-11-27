"""Case 1 role coordination on Gazebo X3 odometry and velocity commands."""
import argparse
import csv
from itertools import combinations
import json
from math import atan2, cos, exp, sin, sqrt
from pathlib import Path
import sys
from time import monotonic, perf_counter

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

import dmpc_core as base
from casadi_backend import CasadiBackend
from engineering_coordination import EngineeringEvents

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from x3_pair_demo import Aircraft, FOLLOWER_LAYOUT
from x3_altitude_demo import TARGET_Z, HOLD_SECONDS, Z_TOLERANCE
from x3_dmpc import VELOCITY_TIME_CONSTANT

SPACE = 0.01
CLOCK = 3.0
SPEED = SPACE / CLOCK
PRE_ROLL = 2.6
SIM_STEP = CLOCK * base.DT
TARGET_REGION = np.array([5500., 0.])
SPEED_LIMIT = SPEED * base.V_MAX
VELOCITY_CHANGE = 1 - exp(-SIM_STEP / VELOCITY_TIME_CONSTANT)
INDEX = {uid: i for i, uid in enumerate(base.IDS)}
PREFIXES = ('/x3',) + tuple(prefix for _, prefix, _ in FOLLOWER_LAYOUT)
PAIRS = tuple(combinations(range(len(PREFIXES)), 2))


def yaw_of(q):
    return atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class CasePlanner:
    def __init__(self):
        initial, _ = base.initialize_events(base.INITIAL)
        self.events = EngineeringEvents(list(initial.agents.values()), seed=base.CASE1_EVENT_SEED)
        self.backend = CasadiBackend(allow_feasible_fallback=True)
        self.backend.model(1)
        self.warm = np.zeros((7, base.N, 2))
        self.previous = None

    def plan(self, state, t):
        for i, uid in enumerate(base.IDS):
            self.events.agents[uid].x, self.events.agents[uid].y = state[i, :2]
        event_start = len(self.events.events)
        graph = self.events.update()['adjacency']
        new_events = self.events.events[event_start:]
        if self.previous is None:
            estimates = [base.constant_velocity_prediction(s) for s in state]
        else:
            estimates = [base.estimate_from_previous_control(p, s) for p, s in zip(self.previous, state)]
        plans = np.zeros_like(self.warm)
        log_start = len(self.backend.logs)
        log_owners = []
        started = perf_counter()
        for i, uid in enumerate(base.IDS):
            before = len(self.backend.logs)
            agent = self.events.agents[uid]
            if agent.role == 'leader':
                plans[i] = self.backend.solve_leader(state[i], t, self.warm[i], 7)
            elif agent.role == 'coordinator':
                cells = self.events.component(uid)
                if 'UAV-1' in cells:
                    q, r = cells['UAV-1']
                    anchor = [0., 0.]
                else:
                    root = min(j for j in cells if self.events.agents[j].role == 'coordinator')
                    q, r = cells[root]
                    anchor = [-base.DS, 0.]
                relative = base.DS * np.array([q + r/2, sqrt(3)*r/2])
                mission = np.array([[90*(t+h*base.DT),0.] for h in range(1,base.N+1)])
                mission += np.array(anchor) - relative
                refs = [
                    estimates[INDEX[j]][1:,:2] - base.slot_offset(n)
                    for (c,j),n in self.events.links.items() if c == uid and j in graph[uid]
                ]
                plans[i] = self.backend.solve(state[i], np.array([*refs, mission]), self.warm[i], 'coordinator')
            elif agent.role == 'follower' and agent.coordinator in graph[uid]:
                plans[i] = self.backend.solve_role(
                    state[i], 'follower', self.warm[i], 7, q=agent.slot,
                    coordinator_estimate=estimates[INDEX[agent.coordinator]],
                )
            elif agent.role == 'single' and agent.relay_to:
                delta = np.array(self.events._slot_position(agent.relay_to, agent.relay_slot)) - state[i, :2]
                heading = base.wrap_angle(atan2(delta[1], delta[0])-state[i,3])
                plans[i,:,0] = np.clip((90-state[i,2])/base.DT,base.A_MIN,base.A_MAX)
                plans[i,:,1] = np.clip(heading/base.DT,base.W_MIN,base.W_MAX)
            log_owners.extend([uid] * (len(self.backend.logs) - before))
        spent = 1000 * (perf_counter() - started)
        fallbacks = [(uid, row) for uid, row in zip(log_owners, self.backend.logs[log_start:], strict=True)
                     if not row['success']]
        self.previous = plans
        self.warm = np.concatenate((plans[:,1:],plans[:,-1:]), axis=1)
        # Convert the predicted one-step acceleration into an X3 velocity command.
        new_speed = SPEED * np.clip(
            state[:,2] + plans[:,0,0] * base.DT / VELOCITY_CHANGE, base.V_MIN, base.V_MAX,
        )
        # Correct the steady velocity deficit observed in the X3 step response.
        new_speed = np.minimum(SPEED_LIMIT, new_speed / 0.92)
        commands = [
            (v*cos(heading), v*sin(heading), plan[0,1]/CLOCK)
            for v, heading, plan in zip(new_speed, state[:,3], plans)
        ]
        return commands, new_events, spent, fallbacks


class CaseFlight(Node):
    def __init__(self):
        super().__init__('x3_case1_gazebo')
        self.aircraft = tuple(Aircraft(self, prefix) for prefix in PREFIXES)
        self.planner = CasePlanner()

    def prepare(self):
        start = monotonic()
        while not all(a.odom is not None and a.subscription.get_publisher_count() == 1
                      and a.command.get_subscription_count() == 1 for a in self.aircraft):
            if monotonic() - start > 15:
                raise RuntimeError('Seven independent Gazebo bridges were not discovered')
            rclpy.spin_once(self, timeout_sec=0.1)
        if any(a.odom.header.frame_id != 'world' for a in self.aircraft):
            raise RuntimeError('Invalid odometry frame for Case 1')

    def run(self, folder):
        self.prepare()
        expected = []
        with (HERE/'seed7_events.csv').open(encoding='utf-8-sig',newline='') as stream:
            expected = [(row['event'],tuple(json.loads(row['agents']))) for row in csv.DictReader(stream)]
        actual = []
        folder.mkdir(parents=True, exist_ok=False)
        initial_xy_error = None
        t0 = None
        previous_step = -1
        hold_start = stable_since = rollout_start = None
        reached = False
        commands = [(0.,0.,0.)]*7
        next_tick = 0.0
        min_sep = float('inf')
        solve_times, fallbacks = [], 0
        final = {}
        with (folder/'states.csv').open('w',newline='') as state_file, \
             (folder/'metrics.csv').open('w',newline='') as metric_file, \
             (folder/'events.csv').open('w',newline='') as event_file,              (folder/'solver_fallbacks.csv').open('w',newline='') as fallback_file:
            states, metrics, events = (csv.writer(stream) for stream in (state_file,metric_file,event_file))
            states.writerow(('gz_elapsed_s','case_time_s','id','x_m','y_m','z_m','speed_m_s','yaw_rad',
                             'role','coordinator','slot','cmd_vx_m_s','cmd_vy_m_s','cmd_yaw_rate_rad_s'))
            metrics.writerow(('gz_elapsed_s','case_time_s','min_spacing_m','closest_pair','max_slot_error_m',
                              'speed_error_m_s','max_yaw_deg','connected_count','solver_wall_ms','feasible_fallbacks',
                              'leader_goal_distance_m','max_height_error_m'))
            events.writerow(('gz_elapsed_s','case_time_s','event','agents','status','detail','slot_links'))
            fallback_log = csv.writer(fallback_file)
            fallback_log.writerow(('gz_elapsed_s','case_time_s','id','role','status','attempts','solver_wall_ms'))
            started = monotonic()
            while monotonic() - started < 325:
                rclpy.spin_once(self, timeout_sec=0.1)
                if any(monotonic() - a.received > 0.8 for a in self.aircraft):
                    raise RuntimeError('Gazebo odometry stopped updating')
                stamps = [a.stamp for a in self.aircraft]
                stamp = min(stamps)
                if stamp < next_tick or max(stamps)-stamp > .07:
                    continue
                next_tick = stamp + .095
                pose = [a.odom.pose.pose for a in self.aircraft]
                positions = np.array([[p.position.x,p.position.y,p.position.z] for p in pose])
                heights = max(abs(positions[:,2] - TARGET_Z))
                if t0 is None:
                    if rollout_start is None:
                        if heights > Z_TOLERANCE:
                            hold_start = None
                        elif hold_start is None:
                            hold_start = stamp
                        elif stamp - hold_start >= HOLD_SECONDS:
                            rollout_start = stamp
                            commands = [(v*SPEED*cos(yaw), v*SPEED*sin(yaw),0.) for _,_,v,yaw in base.INITIAL]
                            print('All seven at altitude; initializing Case 1 entry velocity.', flush=True)
                    elif stamp-rollout_start >= PRE_ROLL:
                        t0 = stamp
                        initial_xy_error = float(max(np.linalg.norm(positions[:,:2]-base.INITIAL[:,:2]*SPACE,axis=1)))
                        print(f'Case 1 mission started. Maximum initial XY error: {initial_xy_error:.3f} m',flush=True)
                if t0 is not None:
                    gz_elapsed = stamp-t0
                    step = int((gz_elapsed+1e-8) / SIM_STEP)
                    if step != previous_step:
                        previous_step = step
                        case_t = step*base.DT
                        speed_m = np.array([a.odom.twist.twist.linear.x for a in self.aircraft])
                        yaw = np.array([yaw_of(p.orientation) for p in pose])
                        state = np.column_stack((positions[:,:2]/SPACE,np.clip(speed_m/SPEED,base.V_MIN,base.V_MAX),yaw))
                        commands, effect, wall_ms, missed = self.planner.plan(state,case_t)
                        misses = len(missed)
                        for uid, row in missed:
                            fallback_log.writerow((gz_elapsed,case_t,uid,row['role'],row['status'],
                                                   json.dumps(row['attempts']),1000*row['seconds_excluding_model_build']))
                        if missed:
                            fallback_file.flush()
                        solve_times.append(wall_ms)
                        fallbacks += misses
                        roles = self.planner.events
                        links = sorted((c,j,n) for (c,j),n in roles.links.items())
                        for event in effect:
                            pair = tuple(event['agents'])
                            if event['status']=='handled':
                                actual.append((event['event'], pair))
                            events.writerow((gz_elapsed,case_t,event['event'],json.dumps(pair),event['status'],
                                             event['detail'],json.dumps(links)))
                            print(f'CASE {case_t:.1f}s: {event["event"]} {pair}',flush=True)
                            event_file.flush()
                        nearest, closest = min((float(np.linalg.norm(positions[i]-positions[j])),(base.IDS[i],base.IDS[j]))
                                               for i,j in PAIRS)
                        min_sep = min(min_sep,nearest)
                        slot = max((float(np.linalg.norm(state[INDEX[j],:2]-state[INDEX[c],:2]-base.slot_offset(n)))*SPACE
                                    for c,j,n in links), default=0.)
                        connected = len(roles.component('UAV-1'))
                        goal = float(np.linalg.norm(state[0,:2]-TARGET_REGION))*SPACE
                        vel_error = float(max(abs(speed_m-90*SPEED)))
                        yaw_error = float(max(abs(np.rad2deg(yaw))))
                        final = dict(gazebo_s=gz_elapsed,case_s=case_t,min_sampled_spacing_m=min_sep,
                                     final_slot_error_m=slot,connected_count=connected,
                                     leader_goal_distance_m=goal,peak_solver_ms=max(solve_times),
                                     feasible_fallbacks=fallbacks,final_speed_error_m_s=vel_error,final_yaw_deg=yaw_error,
                                     links=links)
                        metrics.writerow((gz_elapsed,case_t,nearest,json.dumps(closest),slot,vel_error,yaw_error,connected,
                                          wall_ms,misses,goal,heights))
                        for i,uid in enumerate(base.IDS):
                            agent = roles.agents[uid]
                            states.writerow((gz_elapsed,case_t,uid,*positions[i],speed_m[i],yaw[i],agent.role,
                                             agent.coordinator or '',agent.slot or '',*commands[i]))
                        if step % 10 == 0:
                            print(f'Case 1 {case_t:.1f}s: {connected}/7 connected, slot error {slot:.3f} m, '
                                  f'distance to target {goal:.2f} m', flush=True)
                            metric_file.flush()
                            state_file.flush()
                        stable = connected==7 and slot<=0.15 and heights<=0.05 and vel_error<=0.03 and yaw_error<=7
                        if not stable:
                            stable_since = None
                        elif stable_since is None:
                            stable_since = stamp
                        if goal <= .5 and stable_since is not None and stamp-stable_since >= 3:
                            reached = True
                            break
                        if case_t >= 70:
                            break
                for a,(vx,vy,w) in zip(self.aircraft,commands,strict=True):
                    a.send_velocity(vx,vy,SPEED_LIMIT,yaw_rate=w)
        summary = dict(final, start_sim_time_s=t0, end_sim_time_s=stamp, outcome='target_reached' if reached else 'time_limit',
                       initial_xy_error_m=initial_xy_error, events=actual, expected_events=expected,
                       exact_event_sequence=actual==expected)
        (folder/'summary.json').write_text(json.dumps(summary,indent=2))
        print(json.dumps(summary),flush=True)
        return reached and actual==expected


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = CaseFlight()
    try:
        return 0 if node.run(args.output) else 1
    finally:
        for aircraft in node.aircraft:
            aircraft.command.publish(Twist())
            aircraft.command.wait_for_all_acked(Duration(seconds=1.0))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
