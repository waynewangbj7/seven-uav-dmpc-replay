'''Fly the original Case 1 and continue into staggered obstacle avoidance.'''
import argparse
import json
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.signals import SignalHandlerOptions

from run_avoidance import AvoidanceFlight
from run_case1_gazebo import CaseFlight
from case1_mission_stages import fly_stage


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    geometry = json.loads(args.map.read_text())
    node = CaseFlight()
    try:
        if 'road_entry' in geometry:
            if args.output.exists():
                raise FileExistsError(args.output)
            fly_stage(node, args.output, geometry['road_entry'])
        if not node.run(args.output/'formation'):
            return 1
        new_node = AvoidanceFlight(geometry, staggered=True)
        node.destroy_node()
        node = new_node
        node.run(args.output/'avoidance', airborne=True)
        if 'landing_xy' in geometry:
            fly_stage(node, args.output, geometry['landing_xy'], landing=True)
        return 0
    finally:
        for aircraft in node.aircraft:
            aircraft.command.publish(Twist())
            aircraft.command.wait_for_all_acked(Duration(seconds=1))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
