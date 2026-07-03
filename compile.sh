#!/bin/bash

declare -A results

build() {
  local module="$1"
  shift
  echo ""
  echo "========================================"
  echo ">> Building: $module"
  echo "========================================"
  if "$@"; then
    results["$module"]="OK"
    echo ">> $module: OK"
  else
    results["$module"]="FAILED"
    echo ">> $module: FAILED"
  fi
}

build "mavros"            catkin_make --source Experiment/mavros --build build/mavros_bringup
build "msgs"              catkin_make --source Modules/common/msgs --build build/msgs
build "fast_lio2"         catkin_make --source Modules/fast_lio2 --build build/fast_lio2
# build "ego_planner"    catkin_make --source Modules/ego_planner --build build/ego_planner
build "ego_planner_swarm" catkin_make --source Modules/ego_planner_swarm --build build/ego_planner_swarm
build "swarm_control"     catkin_make --source Modules/swarm_control --build build/swarm_control
build "realsense_ros"     catkin_make --source Modules/realsense_ros --build build/realsense_ros

# Print summary
echo ""
echo "============================================"
echo "         BUILD SUMMARY"
echo "============================================"
failed=0
for module in "${!results[@]}"; do
  status="${results[$module]}"
  if [[ "$status" == "OK" ]]; then
    printf "  [OK]   %-25s\n" "$module"
  else
    printf "  [FAIL] %-25s\n" "$module"
    ((failed++))
  fi
done
echo "============================================"
if [[ $failed -eq 0 ]]; then
  echo "  All modules built successfully."
else
  echo "  $failed module(s) failed."
fi
echo "============================================"
exit $failed
