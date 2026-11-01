#!/usr/bin/env bash
# Prepare a forest scene from raw Mid-360/IMU data without starting flight.
set -eo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"

usage() {
  cat <<'HELP'
Usage: bash Simulation/prepare_scene.sh [options]
  --bag PATH          Input Mid-360 bag (default: ~/bagfiles/yxb_20260208-104116.bag).
  --output PATH       New or empty output directory (default: unique artifacts/forest_lio/scene_*).
  --duration SECONDS  Replay duration from first raw sensor record (default: 192.8).
  --rate NUMBER       Bag playback rate (default: 0.5).
  --map-window A B    Map/capture interval from bag start, seconds (default: 40 195).
  --window A B        Reference route interval from bag start, seconds (default: 125 155).
  -h, --help          Show help without loading ROS.
Defaults are specific to yxb_20260208-104116.bag; review windows/topics for other data.
Relative paths refer to the project root.
Environment: ROS_SETUP, LIVOX_WORKSPACE.
HELP
}
die() { printf 'Error: %s\n' "$*" >&2; exit 2; }
need_values() {
    local count="$1"; shift
    [[ $# -gt "$count" ]] || die "$1 needs $count value(s)"
    local value
    for value in "${@:2:$count}"; do
        [[ -n "$value" && "$value" != --* ]] || die "$1 needs $count value(s)"
    done
}
bag="$HOME/bagfiles/yxb_20260208-104116.bag"
output=""
duration=192.8
rate=.5
map_window=(40 195)
route_window=(125 155)
while [[ $# -gt 0 ]]; do
    case "$1" in
        --bag) need_values 1 "$@"; bag="$2"; shift 2 ;;
        --output) need_values 1 "$@"; output="$2"; shift 2 ;;
        --duration) need_values 1 "$@"; duration="$2"; shift 2 ;;
        --rate) need_values 1 "$@"; rate="$2"; shift 2 ;;
        --map-window) need_values 2 "$@"; map_window=("$2" "$3"); shift 3 ;;
        --window) need_values 2 "$@"; route_window=("$2" "$3"); shift 3 ;;
        -h|--help) usage; exit 0 ;;
        *) usage; die "Unknown option: $1" ;;
    esac
done
cd "$PROJECT_ROOT"
bag="$(realpath -m -- "$bag")"
output="$(realpath -m -- "${output:-artifacts/forest_lio/scene_$(date +%Y%m%d_%H%M%S)_$$}")"
[[ -f "$bag" ]] || die "Missing bag: $bag"
if [[ -e "$output" ]]; then
    [[ -d "$output" ]] || die "Output is not a directory: $output"
    [[ -z "$(find "$output" -mindepth 1 -maxdepth 1 -print -quit)" ]] || die "Output must be empty: $output"
fi
for setup in "${ROS_SETUP:-/opt/ros/noetic/setup.bash}" \
"${LIVOX_WORKSPACE:-$HOME/workspace/ws_livox}/devel/setup.bash" \
"$PROJECT_ROOT/devel/setup.bash"; do
    [[ -f "$setup" ]] || die "Missing environment: $setup. Build ROS/Livox/project dependencies first."
    source "$setup"
done
set -u
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=4
export DISABLE_ROS1_EOL_WARNINGS=1
python3 - "$duration" "$rate" "${map_window[@]}" "${route_window[@]}" <<'PY'
import math
import sys
try:
    duration, rate, ma, mb, ra, rb = map(float, sys.argv[1:])
    if not all(math.isfinite(v) for v in (duration, rate, ma, mb, ra, rb)):
        raise ValueError("Parameters must be finite")
    if duration <= 0 or rate <= 0 or not 0 <= ma <= ra < rb <= mb:
        raise ValueError("Require positive duration/rate and 0 <= map-start <= route-start < route-end <= map-end")
except ValueError as exc:
    sys.exit(str(exc))
PY
bag_start="$(python3 -c 'import rosbag,sys; b=rosbag.Bag(sys.argv[1]); print(b.get_start_time()); b.close()' "$bag")"
printf 'Project: %s\nBag: %s\nOutput: %s\n' "$PROJECT_ROOT" "$bag" "$output"
run_step() {
    printf 'Command:'; printf ' %q' "$@"; printf '\n'
    "$@"
}
run_step python3 "$PROJECT_ROOT/Simulation/run.py" audit "$bag" --output "$output/audit"
run_step python3 "$PROJECT_ROOT/Simulation/run.py" replay "$bag" \
--output "$output/replay" --duration "$duration" --rate "$rate" \
--capture-clouds --map-window "${map_window[@]}"
run_step python3 "$PROJECT_ROOT/Simulation/run.py" coverage "$output/replay/observations.bag" \
--output "$output/coverage"
run_step python3 "$PROJECT_ROOT/Simulation/run.py" prepare-scene \
--replay "$output/replay" --coverage "$output/coverage/coverage.npz" \
--window "${route_window[@]}" --bag-start "$bag_start" --output "$output/scene.json"
printf '\nScene: %s/scene.json\nNext (run from project root):\n' "$output"
printf 'bash Simulation/start.sh short --scene %q --template-bag %q\n' "$output/scene.json" "$bag"
