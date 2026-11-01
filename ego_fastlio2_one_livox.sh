#!/bin/bash
# 单机实机启动：默认启用公共坐标 world 模式。
#
# world 模式（默认）：Fast-LIO2 输出 uav1/lio_init，MAVROS 本地 frame 为 uav1/fcu_local，
# swarm_frame_adapter 把两者都换算到同一个 world，规划器改用 world/odom 与 world/cloud。
# 起飞前需静止约 3 秒（适配节点锁定固定变换），/uav1/frame_ready 为 true 后控制器才放行任务。
#
# 回退旧链路（两套独立 world，仅供对比/排障）：
#   WORLD_FRAME=0 ./ego_fastlio2_one_livox.sh
set -e

export DISABLE_ROS1_EOL_WARNINGS=1
source /opt/ros/noetic/setup.bash
source ~/workspace/ws_livox/devel/setup.bash
source ~/workspace/mspace/devel/setup.bash

WORLD_FRAME="${WORLD_FRAME:-1}"
TFMINI_ID="${TFMINI_ID:-0}"
LIDAR_ID="${LIDAR_ID:-192_168_2_181}"

if [ "$WORLD_FRAME" = "1" ]; then
    MAVROS_ARGS="enable_world_frame:=true tfmini_id:=$TFMINI_ID"
    DRIVER_CMD="roslaunch fast_lio swarm_mid360_driver.launch uav_id:=1"
    LIO_CMD="roslaunch fast_lio mapping_mid360.launch uav_id:=1 enable_world_frame:=true lidar_topic:=/uav1/livox/lidar_$LIDAR_ID imu_topic:=/uav1/livox/imu_$LIDAR_ID"
    CONTROL_ARGS="swarm_num:=1 uav_id:=1 enable_world_frame:=true uav_init_x:=0.0 uav_init_y:=0.0 uav_init_z:=0.0 uav_init_yaw:=0.0"
    PLANNER_ARGS="uav_id:=1 enable_world_frame:=true"
else
    MAVROS_ARGS="tfmini_id:=$TFMINI_ID"
    DRIVER_CMD="roslaunch livox_ros_driver2 msg_MID360.launch"
    LIO_CMD="roslaunch fast_lio mapping_mid360.launch"
    CONTROL_ARGS=""
    PLANNER_ARGS=""
fi

# 起飞前检查：world 模式下 /uav1/frame_ready 必须为 true（地面站/任务指令才会被执行）。
gnome-terminal \
--window -e "bash -c 'roslaunch mavros_bringup px4_mavros.launch $MAVROS_ARGS; exec bash'" \
--tab -e "bash -c 'sleep 5; $DRIVER_CMD; exec bash'" \
--tab -e "bash -c 'sleep 15; $LIO_CMD; exec bash'" \
--tab -e "bash -c 'sleep 10; roslaunch prometheus_swarm_control ego_swarm_control.launch $CONTROL_ARGS; exec bash'" \
--tab -e "bash -c 'sleep 30; roslaunch ego_planner real_ego_run.launch $PLANNER_ARGS; exec bash'" \
--tab -e "bash -c 'sleep 15; roslaunch prometheus_swarm_control ego_station.launch; exec bash'" \
--tab -e "bash -c 'sleep 5; roslaunch realsense2_camera rs_camera.launch; exec bash'"
