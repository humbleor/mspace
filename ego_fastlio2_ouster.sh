#!/bin/bash

export DISABLE_ROS1_EOL_WARNINGS=1
source /opt/ros/noetic/setup.bash
source ~/workspace/ws_livox/devel/setup.bash
source ~/workspace/mspace/devel/setup.bash

gnome-terminal \
--window -e 'bash -c "roslaunch mavros_bringup px4_mavros.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; source /home/joey/workspace/ousterOS0_ws/devel/setup.bash; roslaunch ouster_ros driver.launch; exec bash"' \
--tab -e 'bash -c "sleep 15; roslaunch fast_lio 0_os0128.launch; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch prometheus_swarm_control ego_swarm_control.launch; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch ego_planner osfastlio2_run.launch; exec bash"' \
--tab -e 'bash -c "sleep 15; roslaunch prometheus_swarm_control ego_station.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch realsense2_camera rs_camera.launch; exec bash"' \
