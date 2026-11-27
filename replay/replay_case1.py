"""Publish the archived, planar Case 1 timeline for RViz on ROS 2 Jazzy."""

import argparse
import csv
from collections import defaultdict
from hashlib import sha256
import json
from math import cos, hypot, isfinite, pi, remainder, sin
from pathlib import Path
from subprocess import Popen, TimeoutExpired

from replay_controls import Playback, panel_server


VEHICLES = tuple(f"UAV-{i}" for i in range(1, 8))
ROLE_COLORS = {
    "leader": (1.0, 0.72, 0.25),
    "coordinator": (0.41, 0.86, 1.0),
    "follower": (0.35, 0.91, 0.65),
    "single": (0.76, 0.64, 0.98),
}
DEFAULT_ARCHIVE = Path(__file__).resolve().parent.parent / "case-data"
DEFAULT_MESH = Path(__file__).with_name("mavic_style.dae")
ROTOR_MESH = DEFAULT_MESH.with_name("mavic_rotor.dae")
ROTOR_HUBS = ((32.0, 35.0, 1), (32.0, -35.0, -1),
              (-30.0, 32.0, -1), (-30.0, -32.0, 1))
VIEW_FRAME = "formation_view"
CITY_TOPIC = "/formation/city"
DISPLAY_ALTITUDE = 26.0


def read_csv(path):
    with path.open(encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def load_case(archive, seed):
    raw = archive / "work" / "engineering_case1" / f"seed_{seed}"
    display = archive / "outputs" / "matlab" / f"seed_{seed}"
    provenance = json.loads((display / "topology_provenance.json").read_text(encoding="utf-8-sig"))
    for name in ("states.csv", "events.csv", "metrics.csv", "summary.json"):
        if sha256((raw / name).read_bytes()).hexdigest() != provenance["source_sha256"][name]:
            raise ValueError(f"{name} differs from the verified topology source")

    frames = defaultdict(dict)
    for row in read_csv(raw / "states.csv"):
        time = float(row["time"])
        uid = row["agent_id"]
        if uid in frames[time]:
            raise ValueError(f"Duplicate state for {uid} at {time:g} s")
        frames[time][uid] = (float(row["x"]), float(row["y"]), float(row["yaw"]), row["role"])

    times = sorted(frames)
    if len(times) != provenance["frames"] or any(set(frame) != set(VEHICLES) for frame in frames.values()):
        raise ValueError("The state history must contain all seven vehicles at every frame")

    links = defaultdict(list)
    for row in read_csv(display / "topology.csv"):
        time = float(row["time"])
        edge = row["coordinator"], row["neighbor"]
        if time not in frames or any(uid not in frames[time] for uid in edge):
            raise ValueError(f"Invalid assigned link at {time:g} s: {edge}")
        links[time].append(edge)

    metrics = {float(row["time"]): (float(row["min_pair_distance"]), json.loads(row["closest_pair"]))
               for row in read_csv(raw / "metrics.csv")}
    if set(metrics) != set(times):
        raise ValueError("State and metric timestamps do not agree")
    events = defaultdict(list)
    for row in read_csv(raw / "events.csv"):
        time = float(row["time"])
        if time not in frames:
            raise ValueError(f"No state sample for event at {time:g} s")
        events[time].append({"code": row["event"], "agents": json.loads(row["agents"])})
    return times, frames, links, metrics, events


def check_data(times, frames, links, metrics, events):
    for time in times:
        states = [frames[time][uid] for uid in VEHICLES]
        separation = min(
            hypot(x1 - x2, y1 - y2)
            for i, (x1, y1, _, _) in enumerate(states)
            for x2, y2, _, _ in states[i + 1:]
        )
        distance, pair = metrics[time]
        first, second = (frames[time][uid] for uid in pair)
        if (abs(separation - distance) > 1e-8
                or abs(hypot(first[0] - second[0], first[1] - second[1]) - distance) > 1e-8):
            raise ValueError(f"Separation mismatch at {time:g} s")
    closest_time = min(metrics, key=lambda t: metrics[t][0])
    print(f"{len(times)} frames, {len(VEHICLES)} vehicles, {times[-1]:g} s")
    print(f"Min sampled separation {metrics[closest_time][0]:.3f} m at {closest_time:g} s")
    print(f"{sum(map(len, events.values()))} events, {sum(map(len, links.values()))} link records")


def sample_pose(first, second, fraction):
    x, y, yaw, role = first
    return (
        x + (second[0] - x) * fraction,
        y + (second[1] - y) * fraction,
        yaw + remainder(second[2] - yaw, 2 * pi) * fraction,
        role,
    )


def play(case, playback, mesh_uri, mesh_scale, rotor_uri, launch_rviz=False):
    from geometry_msgs.msg import Point, TransformStamped
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from tf2_ros import TransformBroadcaster
    from visualization_msgs.msg import Marker, MarkerArray

    times, frames, links, _, _ = case
    paths = {uid: [Point(x=frames[t][uid][0], y=frames[t][uid][1], z=8.0)
                   for t in times] for uid in VEHICLES}
    ground_goal = [
        Point(x=5500.0 + 50.0 * cos(i * pi / 24),
              y=50.0 * sin(i * pi / 24), z=-46.0)
        for i in range(49)
    ]

    def make_marker(namespace, number, kind, rgb, alpha=1.0):
        item = Marker()
        item.header.frame_id = "map"
        item.ns, item.id, item.type = namespace, number, kind
        item.pose.orientation.w = 1.0
        item.color.r, item.color.g, item.color.b = rgb
        item.color.a = alpha
        return item

    scenery = []

    blocks = defaultdict(list)

    def block(layer, x, y, z, length, width, height, color):
        blocks[(layer, length, width, height, color)].append(
            Point(x=float(x), y=float(y), z=float(z)))

    block("Ground", 2300, 0, -66, 12600, 5800, 20, (0.39, 0.45, 0.42))
    block("Ground", 2300, -25, -55.5, 9900, 660, 0.8, (0.36, 0.47, 0.38))
    asphalt = (0.27, 0.31, 0.35)
    pavement = (0.47, 0.51, 0.52)
    for y in (320, 645, 970, -515, -840, -1165):
        block("Roads", 2300, y, -54.5, 9900, 48, 1.0, asphalt)
        for x in range(-2200, 6701, 80):
            block("Road markings", x, y, -53.6, 30, 2.5, 0.35, (0.76, 0.75, 0.64))
    for x in range(-2050, 6900, 560):
        for y, length in ((645, 700), (-1000, 380)):
            block("Roads", x, y, -53.9, 40, length, 1.0, asphalt)

    for width, height, color in (
        (146.0, -54.0, (0.49, 0.55, 0.49)),
        (121.0, -53.0, (0.21, 0.43, 0.54)),
    ):
        river = make_marker("River", len(scenery), Marker.LINE_STRIP, color)
        river.scale.x = width
        river.points = [Point(x=float(x), y=-670.0 + 37.0 * sin(x / 420.0), z=height)
                        for x in range(-2700, 7901, 75)]
        scenery.append(river)
    for x in (-370, 1310, 2990, 4670, 6350):
        block("Bridges", x, -665, -50, 55, 342, 4, asphalt)
        for offset in (-25, 25):
            block("Bridges", x + offset, -665, -46, 4, 330, 3, pavement)

    facades = ((0.46, 0.58, 0.65), (0.58, 0.63, 0.66),
               (0.62, 0.56, 0.50), (0.46, 0.51, 0.58))
    for index, x in enumerate(range(-1770, 6621, 560)):
        for row, y in enumerate((485, 812, -1005, -1332)):
            is_park = (index + 2 * row) % 7 == 3
            block("Parks" if is_park else "Sidewalks", x, y, -55, 494, 260, 1.4,
                  (0.38, 0.54, 0.39) if is_park else pavement)
            if is_park:
                block("Park paths", x, y - 15, -53.5, 440, 10, 0.8, (0.69, 0.65, 0.52))
                for offset in range(-180, 181, 60):
                    block("Trees", x + offset, y + 52, -39, 23, 24, 24, (0.23, 0.42, 0.31))
                continue

            for lot, offset in enumerate((-157, 0, 157)):
                cx = x + offset
                cy = y + (20 if lot % 2 else -16)
                height = 38 + 16 * ((index + 3 * row + 2 * lot) % 5)
                if row == 1 and (index + lot) % 4 == 0:
                    height += 62
                length = 98 + 13 * ((row + lot) % 3)
                width = 100 + 15 * ((row + index) % 3)
                color = facades[(index + row + lot) % len(facades)]
                block("Buildings", cx, cy, -54 + height / 2, length, width, height, color)
                block("Roofs", cx, cy, -53 + height, length + 5, width + 5, 4,
                      (0.38, 0.43, 0.48))
                block("Roofs", cx + 10, cy + 11, -44 + height, 37, 42, 12,
                      (0.47, 0.52, 0.57))
                for floor in range(13, height - 3, 16):
                    block("Windows", cx, cy - width / 2 - 0.5, floor - 54,
                          length - 16, 1.4, 3, (0.29, 0.43, 0.54))
                    block("Windows", cx + length / 2 + 0.5, cy, floor - 54,
                          1.4, width - 16, 3, (0.35, 0.49, 0.57))

    block("Goal plaza", 5500, 0, -53.5, 208, 174, 3, (0.40, 0.49, 0.50))
    for y in (-23, 23):
        block("Goal plaza", 5500, y, -50.8, 57, 6, 1, (0.85, 0.87, 0.82))
    block("Goal plaza", 5500, 0, -50.8, 6, 44, 1, (0.85, 0.87, 0.82))

    for number, ((layer, length, width, height, color), points) in enumerate(blocks.items()):
        item = make_marker(layer, number, Marker.CUBE_LIST, color)
        item.scale.x, item.scale.y, item.scale.z = map(float, (length, width, height))
        item.points = points
        scenery.append(item)

    class Replay(Node):
        def __init__(self):
            super().__init__("seven_uav_case1_replay")
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.publisher = self.create_publisher(MarkerArray, "/formation/markers", qos)
            self.city = self.create_publisher(MarkerArray, CITY_TOPIC, qos)
            self.city.publish(MarkerArray(markers=scenery))
            self.camera_tf = TransformBroadcaster(self)
            self.create_timer(0.05, self.publish_frame)

        def publish_frame(self):
            playhead, index, _ = playback.sample()
            time = times[index]
            next_time = times[min(index + 1, len(times) - 1)]
            fraction = (playhead - time) / (next_time - time) if next_time > time else 0.0
            poses = {uid: sample_pose(frames[time][uid], frames[next_time][uid], fraction)
                     for uid in VEHICLES}
            stamp = self.get_clock().now().to_msg()
            messages = []
            center_x = sum(pose[0] for pose in poses.values()) / len(VEHICLES)
            center_y = sum(pose[1] for pose in poses.values()) / len(VEHICLES)
            view = TransformStamped()
            view.header.frame_id = "map"
            view.header.stamp = stamp
            view.child_frame_id = VIEW_FRAME
            view.transform.translation.x = center_x
            view.transform.translation.y = center_y
            view.transform.rotation.w = 1.0
            self.camera_tf.sendTransform(view)

            def marker(namespace, number, kind, rgb, alpha=1.0):
                item = make_marker(namespace, number, kind, rgb, alpha)
                item.header.stamp = stamp
                messages.append(item)
                return item

            def mesh_marker(namespace, number, uri, position, heading, color, use_materials):
                item = marker(namespace, number, Marker.MESH_RESOURCE,
                              (0.0, 0.0, 0.0) if use_materials else color,
                              0.0 if use_materials else 1.0)
                item.mesh_resource = uri
                item.mesh_use_embedded_materials = use_materials
                item.pose.position = Point(x=position[0], y=position[1], z=position[2])
                item.pose.orientation.z = sin(heading / 2)
                item.pose.orientation.w = cos(heading / 2)
                item.scale.x = item.scale.y = item.scale.z = mesh_scale

            # ponytail: frame-rate-friendly display spin. A physics model would supply motor speed.
            phase = 2.0 * pi * ((playhead * 3.2) % 1.0)
            for number, uid in enumerate(VEHICLES, start=1):
                x, y, yaw, role = poses[uid]
                color = ROLE_COLORS[role]
                trail = marker("Trails", number, Marker.LINE_STRIP, color, 0.42)
                trail.scale.x = 4.0
                trail.points = paths[uid][:index + 1] + [Point(x=x, y=y, z=8.0)]

                mesh_marker("Aircraft", number, mesh_uri, (x, y, DISPLAY_ALTITUDE),
                            yaw, color, rotor_uri is not None)
                if rotor_uri is not None:
                    c, s = cos(yaw), sin(yaw)
                    for rotor_index, (hub_x, hub_y, direction) in enumerate(ROTOR_HUBS):
                        position = (
                            x + mesh_scale * (c * hub_x - s * hub_y),
                            y + mesh_scale * (s * hub_x + c * hub_y),
                            DISPLAY_ALTITUDE + mesh_scale * 5.4,
                        )
                        rotor_id = (number - 1) * len(ROTOR_HUBS) + rotor_index
                        heading = yaw + direction * phase + rotor_index * pi / 7
                        mesh_marker("Rotors", rotor_id, rotor_uri, position, heading, color, True)

                label = marker("Names", number, Marker.TEXT_VIEW_FACING, color)
                label.pose.position = Point(x=x, y=y + 59.0, z=51.0)
                label.scale.z = 36.0
                label.text = uid

            assignments = marker("Assigned links", 0, Marker.LINE_LIST, (0.50, 0.78, 1.0), 0.75)
            assignments.scale.x = 5.0
            assignments.points = [Point(x=poses[uid][0], y=poses[uid][1], z=8.0)
                                  for edge in links[time] for uid in edge]

            goal = marker("Goal region", 0, Marker.LINE_STRIP, (1.0, 0.79, 0.40))
            goal.scale.x = 9.0
            goal.points = ground_goal

            self.publisher.publish(MarkerArray(markers=messages))

    rclpy.init()
    node = Replay()
    rviz = None
    try:
        if launch_rviz:
            config = Path(__file__).with_name("case1.rviz").resolve(strict=True)
            print(f"RViz config: {config}", flush=True)
            rviz = Popen(["rviz2", "-d", str(config)])
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        if rviz is not None and rviz.poll() is None:
            rviz.terminate()
            try:
                rviz.wait(timeout=3)
            except TimeoutExpired:
                rviz.kill()
                rviz.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive", type=Path, default=DEFAULT_ARCHIVE)
    parser.add_argument("--seed", type=int, choices=range(7, 12), default=7)
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--mesh", type=Path, default=DEFAULT_MESH, help="Local RViz mesh file, nose along +X")
    parser.add_argument("--mesh-scale", type=float, default=1.0, help="Display-only scale multiplier")
    parser.add_argument("--panel-port", type=int, default=8765, help="Local playback panel port")
    parser.add_argument("--rviz", action="store_true", help="Open RViz with the supplied scene configuration")
    parser.add_argument("--check-data", action="store_true", help="Check archive consistency without ROS")
    args = parser.parse_args()
    if not 0.0 < args.speed <= 20.0:
        parser.error("--speed must be greater than 0 and no greater than 20")
    if not isfinite(args.mesh_scale) or args.mesh_scale <= 0.0:
        parser.error("--mesh-scale must be finite and positive")
    if not 1 <= args.panel_port <= 65535:
        parser.error("--panel-port must be between 1 and 65535")
    try:
        case = load_case(args.archive, args.seed)
        if args.check_data:
            check_data(*case)
        else:
            mesh = args.mesh.resolve(strict=True)
            if not mesh.is_file():
                parser.error("--mesh must point to a local mesh file")
            rotor_uri = ROTOR_MESH.resolve(strict=True).as_uri() if mesh == DEFAULT_MESH.resolve() else None
            playback = Playback(case[0], args.speed)
            with panel_server(args.panel_port, playback, case, args.seed) as url:
                print(f"Playback panel: {url}", flush=True)
                play(case, playback, mesh.as_uri(), args.mesh_scale, rotor_uri, args.rviz)
    except (OSError, ValueError, KeyError, ImportError) as error:
        parser.exit(1, f"{error}\n")


if __name__ == "__main__":
    main()
