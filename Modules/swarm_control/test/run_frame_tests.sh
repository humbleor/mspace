#!/bin/bash
cd "$(dirname "$0")/../../.."
# ROS setup hooks read unbound variables, so keep the shell non-strict until they finish.
source /opt/ros/noetic/setup.bash
source devel/setup.bash
set -euo pipefail
mkdir -p artifacts/swarm_world_frame
g++ -std=c++14 -UNDEBUG -I/usr/include/eigen3 -IModules/swarm_control/include \
    Modules/swarm_control/test/frame_transform_test.cpp -o artifacts/swarm_world_frame/frame_transform_test
artifacts/swarm_world_frame/frame_transform_test
python3 Modules/swarm_control/test/launch_config_test.py
PYTHONWARNINGS=ignore::ResourceWarning python3 Modules/swarm_control/test/frame_adapter_ros_test.py
