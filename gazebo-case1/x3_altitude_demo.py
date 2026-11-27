"""Fly the Gazebo X3 at a fixed altitude, with an optional waypoint."""

import csv
from argparse import ArgumentParser
from contextlib import nullcontext
from math import hypot
from pathlib import Path
from time import monotonic

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

TARGET_Z = 1.50
Z_TOLERANCE = 0.05
HOLD_SECONDS = 3.0
TARGET_X = 2.0
TARGET_Y = 0.0
XY_TOLERANCE = 0.10
MAX_HORIZONTAL_SPEED = 0.30


def velocity_in_body_frame(orientation, vx, vy, vz):
    # Inverse rotation by the unit attitude quaternion from odometry.
    q = orientation
    cross_x = q.y * vz - q.z * vy
    cross_y = q.z * vx - q.x * vz
    cross_z = q.x * vy - q.y * vx
    return (
        vx + 2.0 * (q.y * cross_z - q.z * cross_y - q.w * cross_x),
        vy + 2.0 * (q.z * cross_x - q.x * cross_z - q.w * cross_y),
        vz + 2.0 * (q.x * cross_y - q.y * cross_x - q.w * cross_z),
    )


class AltitudeDemo(Node):
    def __init__(self):
        super().__init__('x3_altitude_demo')
        self.position = None
        self.orientation = None
        self.last_odom = None
        self.sim_seconds = 0.0
        self.command = self.create_publisher(Twist, '/x3/cmd_vel', 10)
        self.create_subscription(Odometry, '/x3/odom', self.on_odometry, 10)

    def on_odometry(self, message):
        self.position = message.pose.pose.position
        self.orientation = message.pose.pose.orientation
        self.last_odom = monotonic()
        stamp = message.header.stamp
        self.sim_seconds = stamp.sec + stamp.nanosec * 1e-9

    def run(self, telemetry_file, waypoint):
        start = monotonic()
        while self.position is None or self.command.get_subscription_count() == 0:
            if monotonic() - start >= 8.0:
                raise RuntimeError('X3 odometry or command bridge not available')
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info(f'Start z={self.position.z:.3f} m, target z={TARGET_Z:.2f} m')
        start = monotonic()
        next_tick = start
        next_log = start
        in_band_since = None
        first_sim_time = None
        telemetry = csv.writer(telemetry_file) if telemetry_file is not None else None
        translating = False
        if telemetry is not None:
            fields = ['sim_time_s', 'target_z_m', 'x_m', 'y_m', 'z_m', 'error_z_m', 'cmd_vz_m_s']
            if waypoint:
                fields[-1] = 'cmd_vz_body_m_s'
                fields.extend((
                    'phase', 'target_x_m', 'target_y_m', 'error_xy_m',
                    'cmd_vx_body_m_s', 'cmd_vy_body_m_s', 'cmd_speed_xy_odom_m_s',
                ))
            telemetry.writerow(fields)

        time_limit = 45.0 if waypoint else 35.0
        while monotonic() - start < time_limit:
            rclpy.spin_once(self, timeout_sec=0.1)
            now = monotonic()
            if now - self.last_odom > 0.5:
                raise RuntimeError('X3 odometry stopped updating')
            if now < next_tick:
                continue
            next_tick = now + 0.1

            position = self.position
            error = TARGET_Z - position.z
            twist = Twist()
            twist.linear.z = max(-0.50, min(0.50, 0.8 * error))
            if translating:
                error_x = TARGET_X - position.x
                error_y = TARGET_Y - position.y
                error_xy = hypot(error_x, error_y)
                vx, vy = 0.8 * error_x, 0.8 * error_y
                scale = max(1.0, hypot(vx, vy) / MAX_HORIZONTAL_SPEED)
                vx, vy = vx / scale, vy / scale
                twist.linear.x, twist.linear.y, twist.linear.z = velocity_in_body_frame(
                    self.orientation, vx, vy, twist.linear.z,
                )
            self.command.publish(twist)

            if telemetry is not None:
                if first_sim_time is None:
                    first_sim_time = self.sim_seconds
                row = [
                    f'{self.sim_seconds - first_sim_time:.3f}', TARGET_Z,
                    position.x, position.y, position.z, error, twist.linear.z,
                ]
                if translating:
                    row.extend((
                        'waypoint', TARGET_X, TARGET_Y, error_xy,
                        twist.linear.x, twist.linear.y, hypot(vx, vy),
                    ))
                elif waypoint:
                    row.extend(('takeoff', '', '', '', 0.0, 0.0, 0.0))
                telemetry.writerow(row)

            if now >= next_log:
                next_log = now + 1.0
                self.get_logger().info(
                    f'x={position.x:.3f}, y={position.y:.3f}, z={position.z:.3f} m, '
                    f'z error={error:+.3f} m'
                )

            if abs(error) > Z_TOLERANCE or (translating and error_xy > XY_TOLERANCE):
                in_band_since = None
            elif in_band_since is None:
                in_band_since = self.sim_seconds
            elif self.sim_seconds - in_band_since >= HOLD_SECONDS:
                if waypoint and not translating:
                    self.get_logger().info(
                        f'Altitude stable; moving toward ({TARGET_X:.2f}, {TARGET_Y:.2f}, '
                        f'{TARGET_Z:.2f}) m at up to {MAX_HORIZONTAL_SPEED:.2f} m/s'
                    )
                    translating = True
                    in_band_since = None
                    continue

                if translating:
                    self.get_logger().info(
                        f'PASS: x={position.x:.3f}, y={position.y:.3f}, z={position.z:.3f} m; '
                        f'XY and height within tolerance for {HOLD_SECONDS:.0f} simulation seconds'
                    )
                else:
                    self.get_logger().info(
                        f'PASS: z={position.z:.3f} m, within {Z_TOLERANCE:.2f} m for '
                        f'{HOLD_SECONDS:.0f} simulation seconds'
                    )
                return
        raise RuntimeError(f'X3 did not reach the target within {time_limit:.0f} seconds')


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('--csv', type=Path, help='Write control samples to a new CSV file.')
    parser.add_argument('--waypoint', action='store_true', help='Fly to the fixed waypoint after takeoff.')
    options, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = AltitudeDemo()
    try:
        output = options.csv.open('x', encoding='utf-8', newline='') if options.csv else nullcontext()
        with output as telemetry_file:
            node.run(telemetry_file, options.waypoint)
    except KeyboardInterrupt:
        return 130
    except (RuntimeError, OSError) as error:
        node.get_logger().error(str(error))
        return 1
    finally:
        node.command.publish(Twist())
        node.command.wait_for_all_acked(Duration(seconds=1.0))
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
