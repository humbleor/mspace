#!/bin/bash
set -e
export DISABLE_ROS1_EOL_WARNINGS=1
source /opt/ros/noetic/setup.bash
source ~/workspace/ws_livox/devel/setup.bash
source ~/workspace/mspace/devel/setup.bash

export ROS_MASTER_URI=http://192.168.1.50:11311
export ROS_IP=192.168.1.40
export LIDAR_ID="${LIDAR_ID:-192_168_2_174}"
UAV2_INIT_X="${UAV2_INIT_X:-0.0}"
UAV2_INIT_Y="${UAV2_INIT_Y:-5.0}"
UAV2_INIT_Z="${UAV2_INIT_Z:-0.0}"
UAV2_INIT_YAW="${UAV2_INIT_YAW:-0.0}"

# Mission bounds in shared world coordinates (m). Set each UAV's site range before joint flight.
UAV2_BOX_MIN_X="${UAV2_BOX_MIN_X:-11.0}"
UAV2_BOX_MIN_Y="${UAV2_BOX_MIN_Y:-11.0}"
UAV2_BOX_MIN_Z="${UAV2_BOX_MIN_Z:-2.0}"
UAV2_BOX_MAX_X="${UAV2_BOX_MAX_X:-22.0}"
UAV2_BOX_MAX_Y="${UAV2_BOX_MAX_Y:-22.0}"
UAV2_BOX_MAX_Z="${UAV2_BOX_MAX_Z:-3.0}"
UAV2_STEP_X="${UAV2_STEP_X:-5.0}"
UAV2_STEP_Y="${UAV2_STEP_Y:-5.0}"
UAV2_STEP_Z="${UAV2_STEP_Z:-2.0}"

until rostopic list >/dev/null 2>&1; do sleep 1; done

# UAV2 is to UAV1's left; both aircraft level and pointing +X. Ground station runs on UAV1 only.
gnome-terminal \
--window -e "bash -c \"sleep 5; roslaunch fast_lio swarm_mid360_driver.launch uav_id:=2; exec bash\"" \
--tab -e "bash -c \"sleep 15; roslaunch fast_lio mapping_mid360.launch uav_id:=2 enable_world_frame:=true lidar_topic:=/uav2/livox/lidar_${LIDAR_ID} imu_topic:=/uav2/livox/imu_${LIDAR_ID}; exec bash\"" \
--tab -e "bash -c \"sleep 10; roslaunch prometheus_swarm_control ego_swarm_control.launch use_bridge:=true swarm_num:=2 uav_id:=2 enable_world_frame:=true uav_init_x:=${UAV2_INIT_X} uav_init_y:=${UAV2_INIT_Y} uav_init_z:=${UAV2_INIT_Z} uav_init_yaw:=${UAV2_INIT_YAW}; exec bash\"" \
--tab -e "bash -c \"sleep 30; roslaunch ego_planner real_ego_run.launch use_bridge:=true uav_id:=2 enable_world_frame:=true box_min_x:=${UAV2_BOX_MIN_X} box_min_y:=${UAV2_BOX_MIN_Y} box_min_z:=${UAV2_BOX_MIN_Z} box_max_x:=${UAV2_BOX_MAX_X} box_max_y:=${UAV2_BOX_MAX_Y} box_max_z:=${UAV2_BOX_MAX_Z} step_X:=${UAV2_STEP_X} step_Y:=${UAV2_STEP_Y} step_Z:=${UAV2_STEP_Z}; exec bash\"" \
--tab -e "bash -c \"sleep 10; roslaunch rosmsg_tcp_bridge bridge.launch uav_id:=2 next_drone_ip:=192.168.1.50 broadcast_ip:=192.168.1.255; exec bash\"" \
--tab -e "bash -c \"sleep 5; roslaunch realsense2_camera rs_camera.launch camera:=uav2_camera tf_prefix:=uav2_camera; exec bash\""
