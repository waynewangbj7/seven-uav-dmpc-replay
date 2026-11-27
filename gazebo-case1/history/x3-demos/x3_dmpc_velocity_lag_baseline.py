"""Low-speed X3 port of the Case 1 CasADi/IPOPT single-shooting solver.

The planar model keeps [x, y, speed, yaw] and the XY tracking cost.
Measured X3 velocity response maps the optimized speed command to actual speed.
"""

from math import atan2, cos, exp, sin
from time import perf_counter

import casadi as ca
import numpy as np

DT = 0.10
N = 12
MIN_SPEED = -0.15
SOLVE_BUDGET = 0.080
VELOCITY_TIME_CONSTANT = 0.62  # x3_velocity_step_20261126.csv

TELEMETRY_FIELDS = (
    'control_source', 'leader_solver_status', 'follower_solver_status',
    'leader_solve_ms', 'follower_solve_ms', 'cycle_solve_ms',
    'leader_vx_cmd_m_s', 'leader_vy_cmd_m_s', 'leader_yaw_rate_cmd_rad_s',
    'follower_vx_cmd_m_s', 'follower_vy_cmd_m_s', 'follower_yaw_rate_cmd_rad_s',
)


class CasadiBackend:
    def __init__(self):
        control = ca.SX.sym('u', 2, N)
        initial = ca.SX.sym('initial', 4)
        targets = ca.SX.sym('targets', 2, N)
        state = initial
        states = [state]
        accelerations = []
        decay = exp(-DT / VELOCITY_TIME_CONSTANT)
        integrated_decay = VELOCITY_TIME_CONSTANT * (1.0 - decay)
        cost = 0
        for k in range(N):
            speed_cmd, yaw_rate = control[0, k], control[1, k]
            x, y, speed, yaw = (state[i] for i in range(4))
            next_speed = decay * speed + (1.0 - decay) * speed_cmd
            a = (next_speed - speed) / DT
            accelerations.append(a)
            distance = speed_cmd * DT + (speed - speed_cmd) * integrated_decay
            state = ca.vertcat(
                x + distance * ca.cos(yaw),
                y + distance * ca.sin(yaw),
                next_speed,
                yaw + DT * yaw_rate,
            )
            states.append(state)
            error = state[:2] - targets[:, k]
            cost += ca.sqrt(ca.dot(error, error) + 0.01**2)
            cost += 0.002 * a**2 + 0.015 * yaw_rate**2

        trajectory = ca.horzcat(*states)
        problem = {
            'x': ca.vec(control),
            'p': ca.vertcat(initial, ca.vec(targets)),
            'f': cost,
            'g': ca.vertcat(*accelerations),
        }
        self.solver = ca.nlpsol('x3_local_ocp', 'ipopt', problem, {
            'error_on_fail': False,
            'print_time': False,
            'ipopt.print_level': 0,
            'ipopt.sb': 'yes',
            'ipopt.max_iter': 45,
            'ipopt.max_wall_time': 0.035,
            'ipopt.tol': 1e-6,
            'ipopt.acceptable_tol': 1e-5,
            'ipopt.bound_relax_factor': 0,
        })
        self.predict = ca.Function('x3_prediction', [control, initial], [trajectory])
        self.lower = np.tile([MIN_SPEED, -0.50], N)

    def solve(self, state, targets, warm, speed_limit):
        started = perf_counter()
        try:
            result = self.solver(
                x0=warm.ravel(), p=np.r_[state, targets.ravel()],
                lbx=self.lower, ubx=np.tile([speed_limit, 0.50], N), lbg=-0.65, ubg=0.65,
            )
            stats = self.solver.stats()
        except RuntimeError:
            return None, None, 'solver_exception', 1000.0 * (perf_counter() - started)

        status = stats['return_status']
        controls = np.asarray(result['x']).reshape(N, 2)
        path = np.asarray(self.predict(controls.T, state)).T
        elapsed_ms = 1000.0 * (perf_counter() - started)
        if not stats['success']:
            return None, None, status, elapsed_ms
        if not (np.all(np.isfinite(controls)) and np.all(np.isfinite(path))):
            return None, None, 'nonfinite_solution', elapsed_ms
        return controls, path, status, elapsed_ms


def measured_state(odometry, speed_limit):
    pose = odometry.pose.pose
    q = pose.orientation
    yaw = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))
    speed = np.clip(odometry.twist.twist.linear.x, MIN_SPEED, speed_limit)
    return np.array([pose.position.x, pose.position.y, speed, yaw])


class PairPlanner:
    def __init__(self, target_x, side_spacing, leader_speed, follower_speed):
        self.backend = CasadiBackend()
        self.goal = np.tile([target_x, 0.0], (N, 1))
        self.offset = np.array([0.0, side_spacing])
        self.leader_speed = leader_speed
        self.follower_speed = follower_speed
        self.leader_warm = np.zeros((N, 2))
        self.follower_warm = np.zeros((N, 2))
        result = self.backend.solve(np.zeros(4), np.zeros((N, 2)), self.leader_warm, leader_speed)
        if result[0] is None:
            raise RuntimeError(f'IPOPT warm-up failed: {result[2]}')

    def plan(self, leader_odom, follower_odom):
        start = perf_counter()
        leader_state = measured_state(leader_odom, self.leader_speed)
        follower_state = measured_state(follower_odom, self.follower_speed)
        leader_u, leader_path, leader_status, leader_ms = self.backend.solve(
            leader_state, self.goal, self.leader_warm, self.leader_speed,
        )
        follower_u, follower_status, follower_ms = None, 'skipped', 0.0
        if leader_u is not None and perf_counter() - start < SOLVE_BUDGET:
            follower_u, _, follower_status, follower_ms = self.backend.solve(
                follower_state, leader_path[1:, :2] + self.offset, self.follower_warm,
                self.follower_speed,
            )
        duration = perf_counter() - start
        timing = (leader_status, follower_status, leader_ms, follower_ms, duration * 1000.0)
        if duration >= SOLVE_BUDGET or 'Time_Exceeded' in (leader_status + follower_status):
            return None, ('fallback_time', *timing)
        if leader_u is None or follower_u is None:
            return None, ('fallback_solver', *timing)

        self.leader_warm = np.vstack((leader_u[1:], leader_u[-1]))
        self.follower_warm = np.vstack((follower_u[1:], follower_u[-1]))
        commands = []
        for state, controls in ((leader_state, leader_u), (follower_state, follower_u)):
            speed = controls[0, 0]
            yaw = state[3]
            commands.append((speed * cos(yaw), speed * sin(yaw), controls[0, 1]))
        return commands, ('dmpc', *timing)
