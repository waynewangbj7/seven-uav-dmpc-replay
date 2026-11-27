#!/usr/bin/env bash
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$root"

usage() {
    echo "Usage: bash run.sh {baseline|a|b|c|dual} [--headless] [--exit-on-finish] [--output DIR]"
}
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
    usage
    exit 0
fi
scenario="${1:-dual}"
if [[ $# -gt 0 ]]; then
    shift
fi
case "$scenario" in
    dual) world="case1_mavic_dual" ;;
    baseline|a|b|c) world="case1_mavic_cycle" ;;
    *) usage >&2; exit 2 ;;
esac
headless=false
exit_on_finish=false
output="$root/runs/${scenario}_$(date +%Y%m%d_%H%M%S)"
while [[ $# -gt 0 ]]; do
    case "$1" in
        --headless) headless=true; shift ;;
        --exit-on-finish) exit_on_finish=true; shift ;;
        --output) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; output="$2"; shift 2 ;;
        *) usage >&2; exit 2 ;;
    esac
done
if [[ -e "$output" ]]; then
    echo "Run output already exists: $output" >&2
    exit 2
fi

source "${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
python="${CASE1_PYTHON:-$root/.venv/bin/python}"
"$python" -c 'import casadi, numpy, rclpy'
bridge="$(ros2 pkg prefix ros_gz_bridge)/lib/ros_gz_bridge/parameter_bridge"
stage="$(mktemp -d -t seven-uav-gazebo.XXXXXX)"
sim_pid=
bridge_pid=
cleanup() {
    if [[ -n "$bridge_pid" ]]; then kill "$bridge_pid" 2>/dev/null || true; fi
    if [[ -n "$sim_pid" ]]; then kill "$sim_pid" 2>/dev/null || true; fi
    wait $bridge_pid $sim_pid 2>/dev/null || true
    rm -rf -- "$stage"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

"$python" "$root/prepare_gazebo.py" "$scenario" "$stage"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-70}"
export GZ_PARTITION="seven_uav_case1_$$"
export GZ_SIM_RESOURCE_PATH="$stage/models:${GZ_SIM_RESOURCE_PATH:-}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
options=(--gui-config "$root/case1/full_gui.config")
if $headless; then options=(-s); fi
gz sim --force-version 8 "${options[@]}" -r -v 2 "$stage/world.sdf" &
sim_pid=$!
"$bridge" --ros-args -p "config_file:=$root/x3_seven_bridge.yaml" &
bridge_pid=$!
timeout 25 gz topic -e -t "/world/$world/stats" -n 1 >/dev/null
if ! $headless; then
    if [[ "$(gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --timeout 10000 --req 'data: "x3_right"')" == *'data: true'* ]]; then
        gz service -s /gui/follow/offset --reqtype gz.msgs.Vector3d --reptype gz.msgs.Boolean --timeout 4000 --req 'x: -4 y: -10 z: 15' >/dev/null || true
    fi
fi
echo "Flight logs: $output"
"$python" "$root/case1/run_case1_full.py" --map "$root/scenes/$scenario.json" --output "$output"
if ! $headless && ! $exit_on_finish; then wait "$sim_pid"; fi