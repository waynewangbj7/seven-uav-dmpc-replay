"""Track a map-based bypass with the Case 1 terminal formation."""
import argparse
import csv
from itertools import combinations
import json
from pathlib import Path
import sys
from time import monotonic

import numpy as np
import rclpy
from geometry_msgs.msg import Quaternion, Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

HERE = Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent))
from x3_pair_demo import Aircraft, FOLLOWER_LAYOUT
from x3_altitude_demo import TARGET_Z, Z_TOLERANCE, velocity_in_body_frame

PREFIXES = ('/x3',) + tuple(prefix for _, prefix, _ in FOLLOWER_LAYOUT)
PAIRS = tuple(combinations(range(7),2))
IDS = tuple(f'UAV-{i}' for i in range(1,8))
XY_BOUND = .10
SPEED = .36
FORWARD_SPEED = .19
DETOUR_WIDTH = 1.42
RAMP_LENGTH = 1.47


def smoothstep(fraction):
    fraction = np.clip(fraction, 0., 1.)
    return fraction**2*(3.-2.*fraction)


class AvoidanceFlight(Node):
    def __init__(self, geometry, staggered):
        super().__init__('case1_static_obstacle')
        self.aircraft = tuple(Aircraft(self,prefix) for prefix in PREFIXES)
        self.offsets = np.array(geometry['offsets'],dtype=float)
        self.route = np.array(geometry['route'],dtype=float)
        self.obstacles = geometry['obstacles']
        self.radius = geometry['body_radius']
        self.staggered = staggered
        self.origin = np.zeros(2)
        if staggered:
            requested = geometry.get('bypass_obstacles', ['orange_container'])
            self.detours = []
            previous_end = -float('inf')
            for name in requested:
                box = next(item for item in self.obstacles if item['name'] == name)
                x, y, _ = box['center']
                width, depth, _ = box['size']
                entry = x-width/2-.77
                exit = x+width/2+.77
                affected = abs(self.offsets[:, 1]-y) < depth/2+self.radius+.22
                if not np.any(affected):
                    raise ValueError(f'No nominal lane intersects {name}')
                lane_x = self.offsets[affected, 0]
                window_start = entry-RAMP_LENGTH-max(lane_x)
                window_end = exit+RAMP_LENGTH-min(lane_x)
                if window_start <= previous_end:
                    raise ValueError(f'The detour for {name} overlaps the preceding detour')
                self.detours.append((name,entry,exit,affected))
                print(f'BYPASS PLAN {name}: {", ".join(u for u, active in zip(IDS,affected) if active)} '
                      f'| leader x {window_start:.2f} to {window_end:.2f} m',flush=True)
                previous_end = window_end
            self.finish_x = previous_end+.65

    def staggered_targets(self, progress):
        targets = self.offsets+self.origin
        targets[:, 0] += progress
        x = targets[:, 0]
        for _, entry, exit, affected in self.detours:
            enter = smoothstep((x-entry+RAMP_LENGTH)/RAMP_LENGTH)
            leave = smoothstep((exit+RAMP_LENGTH-x)/RAMP_LENGTH)
            targets[affected, 1] += DETOUR_WIDTH*(enter*leave)[affected]
        return targets

    def prepare(self):
        started = monotonic()
        while not all(a.odom is not None and a.subscription.get_publisher_count()==1
                      and a.command.get_subscription_count()==1 for a in self.aircraft):
            if monotonic()-started > 12:
                raise RuntimeError('Expected seven independent X3 bridge endpoints')
            rclpy.spin_once(self,timeout_sec=.1)
        if any(a.odom.header.frame_id != 'world' for a in self.aircraft):
            raise RuntimeError('Expected one common world frame')

    def nearest_obstacle(self,xyz):
        nearest = (float('inf'),'',0)
        for obstacle in self.obstacles:
            outside = np.maximum(np.abs(xyz-obstacle['center'])-.5*np.array(obstacle['size']),0.)
            distances = np.linalg.norm(outside,axis=1)
            i = int(np.argmin(distances))
            nearest = min(nearest,(float(distances[i])-self.radius,obstacle['name'],i))
        return nearest

    def run(self, out, airborne=False):
        self.prepare()
        if airborne:
            position = self.aircraft[0].odom.pose.pose.position
            self.origin = np.array([position.x, position.y])
        out.mkdir(parents=True,exist_ok=False)
        phase = 0 if airborne else -1
        first_stamp = hold = flight_start = None
        next_tick = 0.
        minimum_clearance = float('inf')
        minimum_spacing = float('inf')
        peak_formation = 0.
        last = {}
        with (out/'metrics.csv').open('w',newline='') as m_stream, \
             (out/'states.csv').open('w',newline='') as s_stream:
            metrics, states = csv.writer(m_stream), csv.writer(s_stream)
            metrics.writerow(('sim_time_s','waypoint','nearest_body_clearance_m','obstacle','aircraft',
                              'min_pair_spacing_m','formation_error_m','goal_error_m','height_error_m'))
            states.writerow(('sim_time_s','waypoint','id','x_m','y_m','z_m','target_x_m','target_y_m'))
            started = monotonic()
            while monotonic()-started < 150:
                rclpy.spin_once(self,timeout_sec=.1)
                if any(monotonic()-a.received > .75 for a in self.aircraft):
                    raise RuntimeError('Odometry became stale')
                stamps = [a.stamp for a in self.aircraft]
                stamp = min(stamps)
                if stamp < next_tick or max(stamps)-stamp > .07:
                    continue
                next_tick = stamp+.095
                if first_stamp is None:
                    first_stamp = stamp
                    if airborne:
                        flight_start = stamp
                        print('Continuing Case 1 into individual bypasses', flush=True)
                time = stamp-first_stamp
                xyz = np.array([[p.x,p.y,p.z] for p in (a.odom.pose.pose.position for a in self.aircraft)])
                if self.staggered:
                    finish = self.finish_x-self.origin[0]
                    progress = 0. if flight_start is None else min(
                        finish, FORWARD_SPEED*(stamp-flight_start))
                    targets = self.staggered_targets(progress)
                    if phase == 0 and progress == finish:
                        phase = len(self.route)-1
                else:
                    point = np.zeros(2) if phase < 0 else self.route[phase]
                    targets = self.offsets+point
                errors = targets-xyz[:,:2]
                distance = float(max(np.linalg.norm(errors,axis=1)))
                shape = float(max(np.linalg.norm((xyz[:,:2]-xyz[0,:2])-self.offsets,axis=1)))
                z_error = float(max(abs(xyz[:,2]-TARGET_Z)))
                spacing = min(float(np.linalg.norm(xyz[i]-xyz[j])) for i,j in PAIRS)
                clearance, obstacle, i = self.nearest_obstacle(xyz)
                minimum_clearance = min(minimum_clearance,clearance)
                minimum_spacing = min(minimum_spacing,spacing)
                peak_formation = max(peak_formation,shape)
                if clearance < .10 or spacing < .8:
                    raise RuntimeError(f'Clearance guard triggered: obstacle {clearance:.3f}, separation {spacing:.3f} m')
                if phase == -1:
                    errors = np.zeros_like(errors)
                for aircraft,(dx,dy) in zip(self.aircraft,errors,strict=True):
                    aircraft.send_velocity(.85*dx,.85*dy,SPEED)
                metrics.writerow((time,phase,clearance,obstacle,IDS[i],spacing,shape,distance,z_error))
                for j,uid in enumerate(IDS):
                    states.writerow((time,phase,uid,*xyz[j],*targets[j]))
                body_velocity = [
                    a.odom.twist.twist.linear for a in self.aircraft
                ]
                horizontal_speed = max(np.hypot(v.x,v.y) for v in body_velocity)
                inside = z_error<=Z_TOLERANCE and distance<=XY_BOUND and horizontal_speed<=.065
                if self.staggered and phase == 0:
                    continue
                if not inside:
                    hold = None
                elif hold is None:
                    hold = stamp
                elif stamp-hold >= (3.0 if phase in (-1,len(self.route)-1) else .65):
                    if phase == len(self.route)-1:
                        if self.staggered and shape > XY_BOUND:
                            raise RuntimeError(f'Formation has not recovered: {shape:.3f} m')
                        last=dict(outcome='arrived',elapsed_s=time,start_sim_time_s=first_stamp,
                                  min_body_clearance_m=minimum_clearance,min_inter_uav_m=minimum_spacing,
                                  peak_formation_error_m=peak_formation,final_formation_error_m=shape,
                                  final_goal_error_m=distance,final_height_error_m=z_error,
                                  route=self.route.tolist())
                        if self.staggered:
                            last.pop('route')
                            last['offsets'] = self.offsets.tolist()
                            last['origin'] = self.origin.tolist()
                            last['detours'] = {
                                name: [uid for uid, active in zip(IDS, affected, strict=True) if active]
                                for name, _, _, affected in self.detours
                            }
                            last['detouring_aircraft'] = [
                                uid for uid in IDS if any(uid in group for group in last['detours'].values())
                            ]
                        break
                    phase += 1
                    hold = None
                    if self.staggered:
                        flight_start = stamp
                        print('Starting individual bypasses along the open side of the box',flush=True)
                    else:
                        print(f'Waypoint {phase+1}/{len(self.route)}: {self.route[phase].tolist()}',flush=True)
                    m_stream.flush()
                    s_stream.flush()
        if not last:
            raise RuntimeError('The formation did not finish the bypass')
        (out/'summary.json').write_text(json.dumps(last,indent=2))
        print('PASS',json.dumps(last),flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--staggered',action='store_true')
    args=parser.parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=AvoidanceFlight(json.loads(args.map.read_text()),args.staggered)
    try:
        node.run(args.output)
    finally:
        for aircraft in node.aircraft:
            aircraft.command.publish(Twist())
            aircraft.command.wait_for_all_acked(Duration(seconds=1))
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
