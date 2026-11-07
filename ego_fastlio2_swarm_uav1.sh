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
UAV1_INIT_X="${UAV1_INIT_X:-0.0}"
UAV1_INIT_Y="${UAV1_INIT_Y:-0.0}"
UAV1_INIT_Z="${UAV1_INIT_Z:-0.0}"
UAV1_INIT_YAW="${UAV1_INIT_YAW:-0.0}"

# Mission bounds in shared world coordinates (m). Set each UAV's site range before joint flight.
UAV1_BOX_MIN_X="${UAV1_BOX_MIN_X:-0.0}"
UAV1_BOX_MIN_Y="${UAV1_BOX_MIN_Y:-0.0}"
UAV1_BOX_MIN_Z="${UAV1_BOX_MIN_Z:-2.0}"
UAV1_BOX_MAX_X="${UAV1_BOX_MAX_X:-16.0}"
UAV1_BOX_MAX_Y="${UAV1_BOX_MAX_Y:-16.0}"
UAV1_BOX_MAX_Z="${UAV1_BOX_MAX_Z:-3.0}"
UAV1_STEP_X="${UAV1_STEP_X:-5.0}"
UAV1_STEP_Y="${UAV1_STEP_Y:-5.0}"
UAV1_STEP_Z="${UAV1_STEP_Z:-2.0}"

until rostopic list >/dev/null 2>&1; do sleep 1; done

# Common world: UAV1=(0,0,0), UAV2=(0,5,0), both headings +X.
gnome-terminal \
--window -e "bash -c \"sleep 5; roslaunch fast_lio swarm_mid360_driver.launch uav_id:=1; exec bash\"" \
--tab -e "bash -c \"sleep 15; roslaunch fast_lio mapping_mid360.launch uav_id:=1 enable_world_frame:=true lidar_topic:=/uav1/livox/lidar_${LIDAR_ID} imu_topic:=/uav1/livox/imu_${LIDAR_ID}; exec bash\"" \
--tab -e "bash -c \"sleep 10; roslaunch prometheus_swarm_control ego_swarm_control.launch swarm_num:=2 uav_id:=1 enable_world_frame:=true uav_init_x:=${UAV1_INIT_X} uav_init_y:=${UAV1_INIT_Y} uav_init_z:=${UAV1_INIT_Z} uav_init_yaw:=${UAV1_INIT_YAW}; exec bash\"" \
--tab -e "bash -c \"sleep 30; roslaunch ego_planner real_ego_run.launch use_bridge:=true uav_id:=1 enable_world_frame:=true box_min_x:=${UAV1_BOX_MIN_X} box_min_y:=${UAV1_BOX_MIN_Y} box_min_z:=${UAV1_BOX_MIN_Z} box_max_x:=${UAV1_BOX_MAX_X} box_max_y:=${UAV1_BOX_MAX_Y} box_max_z:=${UAV1_BOX_MAX_Z} step_X:=${UAV1_STEP_X} step_Y:=${UAV1_STEP_Y} step_Z:=${UAV1_STEP_Z}; exec bash\"" \
--tab -e "bash -c \"sleep 10; roslaunch rosmsg_tcp_bridge bridge.launch uav_id:=1 next_drone_ip:=192.168.1.40 broadcast_ip:=192.168.1.255; exec bash\"" \
--tab -e "bash -c \"sleep 15; roslaunch prometheus_swarm_control ego_station.launch swarm_num:=2; exec bash\"" \
--tab -e "bash -c \"sleep 5; roslaunch realsense2_camera rs_camera.launch camera:=uav1_camera tf_prefix:=uav1_camera; exec bash\""
