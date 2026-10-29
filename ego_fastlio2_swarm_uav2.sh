#!/bin/bash
set -e
export DISABLE_ROS1_EOL_WARNINGS=1
source /opt/ros/noetic/setup.bash
source ~/workspace/ws_livox/devel/setup.bash
source ~/workspace/mspace/devel/setup.bash

export ROS_MASTER_URI=http://192.168.1.50:11311
export ROS_IP=192.168.1.40
export LIDAR_ID="${LIDAR_ID:-192_168_2_174}"
# Export for the child shells opened by gnome-terminal.
export UAV2_INIT_Y="${UAV2_INIT_Y:-5.0}"
until rostopic list >/dev/null 2>&1; do sleep 1; done

# UAV2 is to UAV1's left; both aircraft level and pointing +X. Ground station runs on UAV1 only.
gnome-terminal \
--window -e 'bash -c "sleep 5; roslaunch fast_lio swarm_mid360_driver.launch uav_id:=2; exec bash"' \
--tab -e 'bash -c "sleep 15; roslaunch fast_lio mapping_mid360.launch uav_id:=2 enable_world_frame:=true lidar_topic:=/uav2/livox/lidar_$LIDAR_ID imu_topic:=/uav2/livox/imu_$LIDAR_ID; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch prometheus_swarm_control ego_swarm_control.launch swarm_num:=2 uav_id:=2 enable_world_frame:=true uav_init_x:=0.0 uav_init_y:=$UAV2_INIT_Y uav_init_z:=0.0 uav_init_yaw:=0.0; exec bash"' \
--tab -e 'bash -c "sleep 30; roslaunch ego_planner real_ego_run.launch use_bridge:=true uav_id:=2 enable_world_frame:=true; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch rosmsg_tcp_bridge bridge.launch uav_id:=2 next_drone_ip:=192.168.1.50 broadcast_ip:=192.168.1.255; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch realsense2_camera rs_camera.launch camera:=uav2_camera tf_prefix:=uav2_camera; exec bash"'
