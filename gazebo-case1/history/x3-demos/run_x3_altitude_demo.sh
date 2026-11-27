#!/usr/bin/env bash
set -e
source "${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"

world=single_x3
bridge=x3_bridge
controller=x3_altitude_demo
default_domain=42
python_exec=python3
controller_options=()
if [[ "${1:-}" == --trio || "${1:-}" == --five || "${1:-}" == --seven ]]; then
    case "$1" in
        --trio) world=x3_trio; default_domain=53 ;;
        --five) world=x3_five; default_domain=55 ;;
        --seven) world=x3_seven; default_domain=57 ;;
    esac
    bridge="${world}_bridge"
    controller=x3_pair_demo
    python_exec="${CASE1_PYTHON:-$PWD/../.venv/bin/python}"
    controller_options=("$1" --dmpc)
    export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
    shift
elif [[ "${1:-}" == --pair ]]; then
    world=x3_pair
    bridge=x3_pair_bridge
    controller=x3_pair_demo
    default_domain=46
    shift
    if [[ "${1:-}" == --dmpc ]]; then
        python_exec="${CASE1_PYTHON:-$PWD/../.venv/bin/python}"
        default_domain=48
        export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
    fi
fi

# Keep this demonstration separate from other open Gazebo worlds.
export GZ_PARTITION="${world}_demo_$$"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$default_domain}"

gz sim --force-version 8 -r -v 1 "$PWD/$world.sdf" &
sim_pid=$!
/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge --ros-args \
    -p "config_file:=$PWD/$bridge.yaml" &
bridge_pid=$!

cleanup() {
    kill "$bridge_pid" "$sim_pid" 2>/dev/null || true
    wait "$bridge_pid" "$sim_pid" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

timeout 20 gz topic -e -t "/world/$world/stats" -n 1 >/dev/null
"$python_exec" "$PWD/../$controller.py" "${controller_options[@]}" "$@"
printf '%s\n' 'X3 demo passed. Close the Gazebo window to end the demo.'
wait "$sim_pid"
