#!/usr/bin/env bash
# Load the project environment, run one isolated simulation, and save its report.
set -eo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
usage() {
  cat <<'HELP'
Usage: bash Simulation/start.sh [full|small|short|hover|test|build] [options]
  full   Default: original generateWps 15-point grid, Gazebo + RViz.
  small  Six-point grid.   short  Takeoff + first scene goal.
  hover  PX4/physics only. test  All simulation tests. build  PX4/plugin build.
Options for flight modes (relative paths use the project root):
  --scene PATH          Forest scene JSON.
  --output PATH         Empty output directory; default generates a unique one.
  --speed NUMBER        Simulation speed factor (default 0.5).
  --duration SECONDS    Simulation-time limit (mode-dependent default).
  --template-bag PATH   Mid-360 bag used for scanning patterns.
  --waypoints PATH      Custom grid JSON for full/small modes.
  --instance ID         Isolated PX4 instance (default 20).
  --gui both|gazebo|rviz|none   Default both; --headless selects none.
  --no-keep-open        Exit after success; default pauses with GUI for inspection.
  --keep-open           Keep successful GUI run open.
  --dry-run             Check inputs and print command; do not launch processes.
  -h, --help            Show this help without loading ROS.
Environment: ROS_SETUP, LIVOX_WORKSPACE, PX4_SOURCE (build source directory).
HELP
}
die() { printf 'Error: %s\n' "$*" >&2; exit 2; }
need_value() { [[ $# -ge 2 && -n "$2" && "$2" != --* ]] || die "$1 needs a value"; }
mode=full
if [[ $# -gt 0 && "$1" != -* ]]; then mode="$1"; shift; fi
case "$mode" in full|small|short|hover|test|build) ;; *) usage; die "Unknown mode: $mode" ;; esac
scene="$PROJECT_ROOT/artifacts/forest_lio/route_125_155.json"
bag="$HOME/bagfiles/yxb_20260208-104116.bag"
speed=.5; duration=""; output=""; waypoints=""; instance=20; gui=both; keep_open=1; dry_run=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    --scene) need_value "$@"; scene="$2"; shift 2 ;;
    --output) need_value "$@"; output="$2"; shift 2 ;;
    --speed) need_value "$@"; speed="$2"; shift 2 ;;
    --duration) need_value "$@"; duration="$2"; shift 2 ;;
    --template-bag) need_value "$@"; bag="$2"; shift 2 ;;
    --waypoints) need_value "$@"; waypoints="$2"; shift 2 ;;
    --instance) need_value "$@"; instance="$2"; shift 2 ;;
    --gui) need_value "$@"; gui="$2"; shift 2 ;;
    --headless) gui=none; shift ;;
    --keep-open) keep_open=1; shift ;;
    --no-keep-open) keep_open=0; shift ;;
    --dry-run) dry_run=1; shift ;;
    *) usage; die "Unknown option: $1" ;;
  esac
done
cd "$PROJECT_ROOT"
ros_setup="${ROS_SETUP:-/opt/ros/noetic/setup.bash}"
livox_workspace="${LIVOX_WORKSPACE:-$HOME/workspace/ws_livox}"
for setup in "$ros_setup" "$livox_workspace/devel/setup.bash" "$PROJECT_ROOT/devel/setup.bash"; do
  [[ -f "$setup" ]] || die "Missing environment: $setup. Build ROS/Livox/project dependencies first (./compile.sh)."
  # Generated catkin environment scripts expect unset variables to be allowed.
  source "$setup"
done
set -u
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=4
print_command() { printf 'Command:'; printf ' %q' "$@"; printf '\n'; }
if [[ "$mode" == test || "$mode" == build ]]; then
  if [[ "$mode" == test ]]; then command=(python3 "$PROJECT_ROOT/Simulation/run.py" test)
  else command=(python3 "$PROJECT_ROOT/Simulation/run.py" px4 build --source "${PX4_SOURCE:-$HOME/workspace/PX4_Firmware}"); fi
  print_command "${command[@]}"
  if [[ "$dry_run" == 1 ]]; then exit 0; fi
  exec "${command[@]}"
fi
case "$gui" in both|gazebo|rviz|none) ;; *) die "Invalid GUI mode: $gui" ;; esac
stage=forest
mission=()
case "$mode" in
  full) duration="${duration:-600}"; waypoints="${waypoints:-$PROJECT_ROOT/Simulation/px4_forest/config/grid_route.json}" ;;
  small) duration="${duration:-180}"; waypoints="${waypoints:-$PROJECT_ROOT/Simulation/px4_forest/config/grid_alignment_smoke.json}" ;;
  short) duration="${duration:-90}"; [[ -z "$waypoints" ]] || die "short cannot use --waypoints"; mission=(--goals 1) ;;
  hover) stage=hover; duration="${duration:-40}"; [[ -z "$waypoints" ]] || die "hover cannot use --waypoints" ;;
esac
if [[ -n "$waypoints" ]]; then
  [[ -f "$waypoints" ]] || die "Missing grid: $waypoints"
  mission=(--waypoints "$waypoints")
fi
[[ -x "$PROJECT_ROOT/artifacts/px4_forest/PX4-Autopilot/build/px4_sitl_default/bin/px4" ]] || die 'PX4 is not built. Run: bash Simulation/start.sh build'
[[ -f "$PROJECT_ROOT/artifacts/px4_forest/plant_build/libmspace_forest_plant.so" ]] || die 'Plant plugin is not built. Run: bash Simulation/start.sh build'
python3 - "$stage" "$scene" "$bag" "$speed" "$duration" "$instance" <<'PY'
import json,sys
from pathlib import Path
import rospkg
stage,scene,bag,speed,duration,instance=sys.argv[1:]
assert 0<float(speed)<=1 and float(duration)>10 and 0<=int(instance)<254, "Invalid speed/duration/instance"
for name in ("fast_lio","ego_planner","prometheus_swarm_control","livox_ros_driver2","mavros"):
    rospkg.RosPack().get_path(name)
if stage=="forest":
    scene=Path(scene)
    if not scene.is_file():raise SystemExit("Missing scene: "+str(scene))
    data=json.loads(scene.read_text())
    for key in ("map","coverage"):
        if not Path(data[key]).is_file():raise SystemExit("Missing scene "+key+": "+data[key])
    if not Path(bag).is_file():raise SystemExit("Missing Mid-360 template bag: "+bag)
PY
output="${output:-$PROJECT_ROOT/artifacts/px4_forest/${mode}_$(date +%Y%m%d_%H%M%S)_$$}"
if [[ -e "$output" ]]; then
  [[ -d "$output" ]] || die "Output is not a directory: $output"
  [[ -z "$(find "$output" -mindepth 1 -maxdepth 1 -print -quit)" ]] || die "Output must be empty: $output"
fi
command=(python3 "$PROJECT_ROOT/Simulation/run.py" px4 run --stage "$stage" --output "$output"
         --duration "$duration" --speed "$speed" --instance "$instance" --gui "$gui")
if [[ "$stage" == forest ]]; then command+=(--scene "$scene" --template-bag "$bag" "${mission[@]}"); fi
if [[ "$keep_open" == 1 && "$gui" != none ]]; then command+=(--keep-open); fi
printf 'Project: %s\nMode: %s\nOutput: %s\n' "$PROJECT_ROOT" "$mode" "$output"
print_command "${command[@]}"
if [[ "$dry_run" == 1 ]]; then exit 0; fi
set +e
"${command[@]}"
run_status=$?
set -e
report_status=0
if [[ -f "$output/result.json" ]]; then
  python3 "$PROJECT_ROOT/Simulation/run.py" px4 report "$output" || report_status=$?
  printf 'Result: %s/result.json\nReport: %s/report.md\n' "$output" "$output"
fi
if [[ "$run_status" -ne 0 ]]; then exit "$run_status"; fi
exit "$report_status"
