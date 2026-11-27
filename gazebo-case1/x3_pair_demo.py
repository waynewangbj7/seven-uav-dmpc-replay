"""Fly two X3 models in lateral formation in a single Gazebo world."""

import csv
from argparse import ArgumentParser
from contextlib import nullcontext
from itertools import combinations
from math import hypot
from pathlib import Path
from time import monotonic

import rclpy
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from x3_altitude_demo import (
    HOLD_SECONDS, MAX_HORIZONTAL_SPEED, TARGET_X, TARGET_Z, XY_TOLERANCE,
    Z_TOLERANCE, velocity_in_body_frame,
)

SIDE_SPACING = 2.0
FOLLOWER_SPEED_LIMIT = 0.40
POSITION_GAIN = 0.8
FOLLOWER_LAYOUT = (
    ('follower', '/x3_follower', SIDE_SPACING),
    ('right', '/x3_right', -SIDE_SPACING),
    ('outer_left', '/x3_outer_left', 2.0 * SIDE_SPACING),
    ('outer_right', '/x3_outer_right', -2.0 * SIDE_SPACING),
    ('edge_left', '/x3_edge_left', 3.0 * SIDE_SPACING),
    ('edge_right', '/x3_edge_right', -3.0 * SIDE_SPACING),
)


class Aircraft:
    def __init__(self, node, prefix):
        self.odom = None
        self.received = None
        self.stamp = None
        self.command = node.create_publisher(Twist, f'{prefix}/cmd_vel', 10)
        self.subscription = node.create_subscription(Odometry, f'{prefix}/odom', self.update, 10)

    def update(self, message):
        self.odom = message
        stamp = message.header.stamp
        self.stamp = stamp.sec + stamp.nanosec * 1e-9
        self.received = monotonic()

    def send_velocity(self, vx, vy, speed_limit, yaw_rate=0.0, *, target_z=TARGET_Z, vertical_limit=0.50):
        scale = max(1.0, hypot(vx, vy) / speed_limit)
        pose = self.odom.pose.pose
        vz = max(-vertical_limit, min(vertical_limit, 0.8 * (target_z - pose.position.z)))
        command = Twist()
        command.linear.x, command.linear.y, command.linear.z = velocity_in_body_frame(
            pose.orientation, vx / scale, vy / scale, vz,
        )
        command.angular.z = yaw_rate
        self.command.publish(command)
        return vx / scale, vy / scale, yaw_rate


class PairDemo(Node):
    def __init__(self, planner=None, layout=FOLLOWER_LAYOUT[:1]):
        demo_names = {
            1: 'x3_pair_demo', 2: 'x3_trio_demo', 4: 'x3_five_demo', 6: 'x3_seven_demo',
        }
        super().__init__(demo_names[len(layout)])
        self.planner = planner
        self.leader = Aircraft(self, '/x3')
        self.aircraft = (self.leader,) + tuple(Aircraft(self, prefix) for _, prefix, _ in layout)
        self.follower = self.aircraft[1]
        self.follower_names = tuple(name for name, _, _ in layout)
        self.offsets = tuple(offset for _, _, offset in layout)
        self.speed_limits = (MAX_HORIZONTAL_SPEED,) + (FOLLOWER_SPEED_LIMIT,) * len(self.offsets)

    def run(self, telemetry_file):
        start = monotonic()
        while not all(
            aircraft.odom is not None
            and aircraft.subscription.get_publisher_count() == 1
            and aircraft.command.get_subscription_count() == 1
            for aircraft in self.aircraft
        ):
            if monotonic() - start >= 10.0:
                raise RuntimeError('Expected one odometry publisher and one command subscriber per aircraft')
            rclpy.spin_once(self, timeout_sec=0.1)

        positions = [aircraft.odom.pose.pose.position for aircraft in self.aircraft]
        leader = positions[0]
        if any(aircraft.odom.header.frame_id != 'world' for aircraft in self.aircraft):
            raise RuntimeError('All odometry topics must use the same world frame')
        if max(position.z for position in positions) > 0.10 or any(
            abs(position.y - leader.y - offset) > 0.10
            for position, offset in zip(positions[1:], self.offsets, strict=True)
        ):
            raise RuntimeError('Expected all aircraft on the ground at their assigned offsets')
        spawn = '; '.join(f'y={position.y:.3f}, z={position.z:.3f}' for position in positions)
        self.get_logger().info(f'Start: {spawn} m')

        telemetry = csv.writer(telemetry_file) if telemetry_file is not None else None
        if telemetry is not None:
            fields = [
                'sim_time_s', 'phase', 'leader_x_m', 'leader_y_m', 'leader_z_m',
                'follower_x_m', 'follower_y_m', 'follower_z_m',
                'follower_target_x_m', 'follower_target_y_m', 'target_z_m',
                'leader_goal_error_xy_m', 'formation_error_xy_m', 'separation_m',
            ]
            if len(self.aircraft) > 2:
                fields.append('follower_error_xy_m')
                for name in self.follower_names[1:]:
                    fields.extend(f'{name}_{field}' for field in (
                        'x_m', 'y_m', 'z_m', 'target_x_m', 'target_y_m', 'error_xy_m',
                    ))
            if self.planner is not None:
                fields.extend(self.planner.telemetry_fields)
            telemetry.writerow(fields)

        first_stamp = None
        next_tick = next_log = monotonic()
        deadline = next_tick + 50.0
        in_band_since = None
        moving = False
        while monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            now = monotonic()
            if any(now - aircraft.received > 0.5 for aircraft in self.aircraft):
                raise RuntimeError('Odometry stopped updating')
            stamps = [aircraft.stamp for aircraft in self.aircraft]
            stamp = min(stamps)
            if now < next_tick or max(stamps) - stamp > 0.06:
                continue
            next_tick = now + 0.1
            if first_stamp is None:
                first_stamp = stamp

            leader_pose = self.leader.odom.pose.pose
            positions = [aircraft.odom.pose.pose.position for aircraft in self.aircraft]
            leader, follower = positions[:2]
            targets = [(leader.x, leader.y + offset) for offset in self.offsets]
            differences = [
                (target_x - position.x, target_y - position.y)
                for position, (target_x, target_y) in zip(positions[1:], targets, strict=True)
            ]
            errors = [hypot(dx, dy) for dx, dy in differences]
            formation_error = max(errors)
            goal_error = hypot(TARGET_X - leader.x, leader.y)
            height_error = max(abs(TARGET_Z - position.z) for position in positions)
            separation = min(
                hypot(a.x - b.x, a.y - b.y, a.z - b.z) for a, b in combinations(positions, 2)
            )

            commands = None
            solver_info = ('takeoff',) + ('',) * (2 * len(self.aircraft) + 1)
            if moving and self.planner is not None:
                commands, solver_info = self.planner.plan(*(aircraft.odom for aircraft in self.aircraft))
            if commands is None and moving:
                # Use the bounded position control law if any local solve fails.
                q = leader_pose.orientation
                conjugate = Quaternion(x=-q.x, y=-q.y, z=-q.z, w=q.w)
                velocity = self.leader.odom.twist.twist.linear
                world_vx, world_vy, _ = velocity_in_body_frame(
                    conjugate, velocity.x, velocity.y, velocity.z,
                )
                commands = [(POSITION_GAIN * (TARGET_X - leader.x), -POSITION_GAIN * leader.y, 0.0)]
                commands.extend(
                    (world_vx + POSITION_GAIN * dx, world_vy + POSITION_GAIN * dy, 0.0)
                    for dx, dy in differences
                )
            if commands is None:
                commands = [(0.0, 0.0, 0.0)] * len(self.aircraft)
            applied = [
                aircraft.send_velocity(vx, vy, speed_limit, yaw_rate=yaw_rate)
                for aircraft, (vx, vy, yaw_rate), speed_limit in zip(
                    self.aircraft, commands, self.speed_limits, strict=True,
                )
            ]

            if telemetry is not None:
                row = (
                    f'{stamp - first_stamp:.3f}', 'formation' if moving else 'takeoff',
                    leader.x, leader.y, leader.z, follower.x, follower.y, follower.z,
                    *targets[0], TARGET_Z, goal_error, formation_error, separation,
                )
                if len(self.aircraft) > 2:
                    row += (errors[0],)
                    for position, target, error in zip(positions[2:], targets[1:], errors[1:], strict=True):
                        row += (position.x, position.y, position.z, *target, error)
                if self.planner is not None:
                    row += (*solver_info, *(value for command in applied for value in command))
                telemetry.writerow(row)

            if now >= next_log:
                next_log = now + 1.0
                location = ', '.join(f'x={position.x:.3f}/z={position.z:.3f}' for position in positions)
                self.get_logger().info(f'{location} m, formation error={formation_error:.3f} m')

            if height_error > Z_TOLERANCE or (
                moving and max(goal_error, formation_error) > XY_TOLERANCE
            ):
                in_band_since = None
            elif in_band_since is None:
                in_band_since = stamp
            elif stamp - in_band_since >= HOLD_SECONDS:
                if moving:
                    self.get_logger().info(
                        f'PASS: leader goal={goal_error:.3f} m, formation error={formation_error:.3f} m, '
                        f'height error={height_error:.3f} m; stable for {HOLD_SECONDS:.0f} simulation seconds'
                    )
                    return
                moving = True
                in_band_since = None
                self.get_logger().info('All aircraft at 1.50 m; starting formation translation.')

        raise RuntimeError('The formation did not reach the target within 50 seconds')


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('--csv', type=Path, help='Write a new run CSV.')
    parser.add_argument('--dmpc', action='store_true', help='Use local CasADi optimizers during translation.')
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--trio', dest='count', action='store_const', const=3)
    group.add_argument('--five', dest='count', action='store_const', const=5)
    group.add_argument('--seven', dest='count', action='store_const', const=7)
    parser.set_defaults(count=2)
    options, ros_args = parser.parse_known_args()
    layout = FOLLOWER_LAYOUT[:options.count - 1]
    planner = None
    if options.dmpc:
        from x3_dmpc import FormationPlanner

        offsets = tuple(offset for _, _, offset in layout)
        names = ('leader',) + tuple(name for name, _, _ in layout)
        planner = FormationPlanner(TARGET_X, offsets, MAX_HORIZONTAL_SPEED, FOLLOWER_SPEED_LIMIT, names)
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = PairDemo(planner, layout=layout)
    try:
        output = options.csv.open('x', encoding='utf-8', newline='') if options.csv else nullcontext()
        with output as telemetry_file:
            node.run(telemetry_file)
    except KeyboardInterrupt:
        return 130
    except (RuntimeError, OSError) as error:
        node.get_logger().error(str(error))
        return 1
    finally:
        for aircraft in node.aircraft:
            aircraft.command.publish(Twist())
        for aircraft in node.aircraft:
            aircraft.command.wait_for_all_acked(Duration(seconds=1.0))
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
