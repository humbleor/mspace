#!/bin/bash
set -e
export DISABLE_ROS1_EOL_WARNINGS=1
source /opt/ros/noetic/setup.bash
source ~/workspace/ws_livox/devel/setup.bash
source ~/workspace/mspace/devel/setup.bash

# Start the updated ego_fastlio2_swarm_mavros.sh first.
export ROS_MASTER_URI=http://192.168.1.50:11311
export ROS_IP=192.168.1.50
export LIDAR_ID="${LIDAR_ID:-192_168_2_181}"
until rostopic list >/dev/null 2>&1; do sleep 1; done

# Common world: UAV1=(0,0,0), UAV2=(0,5,0), both headings +X.
# Tree matching is deliberately disabled until fixed-frame alignment is validated.
gnome-terminal \
--window -e 'bash -c "sleep 5; roslaunch fast_lio swarm_mid360_driver.launch uav_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 15; roslaunch fast_lio mapping_mid360.launch uav_id:=1 enable_world_frame:=true lidar_topic:=/uav1/livox/lidar_$LIDAR_ID imu_topic:=/uav1/livox/imu_$LIDAR_ID; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch prometheus_swarm_control ego_swarm_control.launch swarm_num:=2 uav_id:=1 enable_world_frame:=true uav_init_x:=0.0 uav_init_y:=0.0 uav_init_z:=0.0 uav_init_yaw:=0.0; exec bash"' \
--tab -e 'bash -c "sleep 30; roslaunch ego_planner real_ego_run.launch use_bridge:=true uav_id:=1 enable_world_frame:=true; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch rosmsg_tcp_bridge bridge.launch uav_id:=1 next_drone_ip:=192.168.1.40 broadcast_ip:=192.168.1.255; exec bash"' \
--tab -e 'bash -c "sleep 15; roslaunch prometheus_swarm_control ego_station.launch swarm_num:=2; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch realsense2_camera rs_camera.launch camera:=uav1_camera tf_prefix:=uav1_camera; exec bash"'
