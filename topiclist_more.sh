#!/bin/bash
# 录制全部话题，Ctrl+C 结束。

BAG_DIR="${BAG_DIR:-$HOME/data}"
PREFIX="${PREFIX:-flight_$(date +%Y%m%d-%H%M%S)}"
MAX_SIZE="${MAX_SIZE:-4096}"  # 每包最大 MB
MIN_SPACE="${MIN_SPACE:-5G}"  # 最低剩余磁盘空间

mkdir -p "$BAG_DIR" && cd "$BAG_DIR" || exit 1
echo "录制全部话题到 $BAG_DIR/${PREFIX}*.bag（Ctrl+C 结束）"
exec rosbag record -a -b 256 --lz4 --split --size="$MAX_SIZE" -L "$MIN_SPACE" -O "$PREFIX"
