#!/bin/bash

export DISABLE_ROS1_EOL_WARNINGS=1
source /opt/ros/noetic/setup.bash
source ~/workspace/ws_livox/devel/setup.bash
source ~/workspace/mspace/devel/setup.bash

gnome-terminal \
--window -e 'bash -c "sleep 5; roslaunch livox_ros_driver2 msg_MID360.launch; exec bash"' \
--tab -e 'bash -c "sleep 15; roslaunch fast_lio mapping_mid360.launch uav_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch prometheus_swarm_control ego_swarm_control.launch swarm_num:=2 uav_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch ego_planner osfastlio2_simdemo.launch uav_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 15; roslaunch prometheus_swarm_control ego_station.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch realsense2_camera rs_camera.launch; exec bash"'
