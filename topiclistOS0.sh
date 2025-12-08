#!/bin/bash

cd /home/joey/data

if [ ! -d "/home/joey/data" ]; then
    mkdir -p /home/joey/data
fi

rosbag record -b 1024 /ouster/imu /ouster/points /livox/lidar_192_168_2_181 /livox/imu_192_168_2_181 /camera/color/image_raw/compressed /d400/color/image_raw/compressed

# file="/home/joey/data/full_$(date +%Y%m%d-%H%M%S).bag"
# rosbag record -a -b 1024 -O "$file"
