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

            # Match the reference motion and reduce the command as position error closes.
            reference_step = max(1, k)
            reference_velocity = (targets[:, reference_step] - targets[:, reference_step - 1]) / DT
            reference_command = reference_velocity + 0.8 * (targets[:, k] - ca.vertcat(x, y))
            commanded_velocity = speed_cmd * ca.vertcat(ca.cos(yaw), ca.sin(yaw))
            velocity_error = commanded_velocity - reference_command
            cost += 2.0 * ca.dot(velocity_error, velocity_error)

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


class FormationPlanner:
    def __init__(self, target_x, follower_offsets, leader_speed, follower_speed, names):
        self.backend = CasadiBackend()
        self.goal = np.tile([target_x, 0.0], (N, 1))
        self.offsets = [np.array([0.0, offset]) for offset in follower_offsets]
        self.speed_limits = (leader_speed,) + (follower_speed,) * len(follower_offsets)
        self.warm = [np.zeros((N, 2)) for _ in self.speed_limits]
        self.telemetry_fields = (
            'control_source',
            *(f'{name}_solver_status' for name in names),
            *(f'{name}_solve_ms' for name in names),
            'cycle_solve_ms',
            *(f'{name}_{field}' for name in names for field in (
                'vx_cmd_m_s', 'vy_cmd_m_s', 'yaw_rate_cmd_rad_s',
            )),
        )
        result = self.backend.solve(np.zeros(4), np.zeros((N, 2)), self.warm[0], leader_speed)
        if result[0] is None:
            raise RuntimeError(f'IPOPT warm-up failed: {result[2]}')

    def plan(self, *odometry):
        start = perf_counter()
        states = [
            measured_state(odom, speed_limit)
            for odom, speed_limit in zip(odometry, self.speed_limits, strict=True)
        ]
        solutions = []
        statuses = ['skipped'] * len(states)
        solve_ms = [0.0] * len(states)
        leader_path = None
        for i, (state, warm, speed_limit) in enumerate(zip(states, self.warm, self.speed_limits, strict=True)):
            if perf_counter() - start >= SOLVE_BUDGET:
                break
            targets = self.goal if i == 0 else leader_path[1:, :2] + self.offsets[i - 1]
            controls, path, statuses[i], solve_ms[i] = self.backend.solve(state, targets, warm, speed_limit)
            if controls is None:
                break
            if i == 0:
                leader_path = path
            solutions.append(controls)

        duration = perf_counter() - start
        timing = (*statuses, *solve_ms, duration * 1000.0)
        if duration >= SOLVE_BUDGET or any('Time_Exceeded' in status for status in statuses):
            return None, ('fallback_time', *timing)
        if len(solutions) != len(states):
            return None, ('fallback_solver', *timing)

        commands = []
        for i, (state, controls) in enumerate(zip(states, solutions, strict=True)):
            self.warm[i] = np.vstack((controls[1:], controls[-1]))
            speed, yaw_rate = controls[0]
            yaw = state[3]
            commands.append((speed * cos(yaw), speed * sin(yaw), yaw_rate))
        return commands, ('dmpc', *timing)
