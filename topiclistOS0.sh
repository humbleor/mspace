#!/bin/bash

mkdir -p "$HOME/data"
cd "$HOME/data" || exit 1

# 录制指定话题：点云/IMU/相机
rosbag record \
-b 256 \
--split \
--duration=60 \
--regex \
'/ouster/imu|/ouster/points|/livox/lidar_.*|/livox/imu_.*|/camera/color/image_raw/compressed|/d400/color/image_raw/compressed'

# 全量录制
# rosbag record -a -b 1024 -O "$HOME/data/full_$(date +%Y%m%d-%H%M%S).bag"
