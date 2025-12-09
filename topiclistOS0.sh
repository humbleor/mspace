#!/bin/bash

cd ~/data

if [ ! -d "~/data" ]; then
    mkdir -p ~/data
fi

rosbag record -b 1024 --regex /ouster/imu /ouster/points /livox/lidar_.* /livox/imu_.* /camera/color/image_raw/compressed /d400/color/image_raw/compressed

# file="/home/joey/data/full_$(date +%Y%m%d-%H%M%S).bag"
# rosbag record -a -b 1024 -O "$file"
