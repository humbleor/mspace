#!/bin/bash

set -e

export DISABLE_ROS1_EOL_WARNINGS=1
source /opt/ros/noetic/setup.bash
source ~/workspace/ws_livox/devel/setup.bash
source ~/workspace/mspace/devel/setup.bash

# UAV1 (joey) = ROS Master，由 ego_fastlio2_swarm_mavros.sh 启动
export ROS_MASTER_URI=http://192.168.1.50:11311
export ROS_IP=192.168.1.40

# 等 Master 就绪，避免 tabs 起来后 roscore 还没在
until rostopic list >/dev/null 2>&1; do sleep 1; done

UAV2_INIT_X=3.0
# 注意：地面站只在 UAV1 上启动，UAV2 不需要
gnome-terminal \
--window -e 'bash -c "sleep 5; roslaunch livox_ros_driver2 msg_MID360.launch; exec bash"' \
--tab -e 'bash -c "sleep 15; roslaunch fast_lio mapping_mid360.launch uav_id:=2; exec bash"' \
--tab -e 'bash -c "sleep 20; roslaunch drone_detect_lidar drone_detect_lidar.launch drone_id:=2; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch prometheus_swarm_control ego_swarm_control.launch \
    swarm_num:=2 uav_id:=2 uav_init_x:=$UAV2_INIT_X; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch ego_planner real_ego_run.launch uav_id:=2; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch rosmsg_tcp_bridge bridge.launch uav_id:=2 \
    next_drone_ip:=192.168.1.50 broadcast_ip:=192.168.1.255; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch realsense2_camera rs_camera.launch; exec bash"'
