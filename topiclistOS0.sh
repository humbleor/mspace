#!/bin/bash

mkdir -p "$HOME/data"
cd "$HOME/data" || exit 1

# 话题正则（rosbag -e 是整名匹配，必须写到话题名末尾）：
#   /livox/lidar_192_168_2_174        旧单机
#   /uav1/livox/lidar_192_168_2_174   world 模式
#   /ouster/points 与 /os_cloud_node_1/points  Ouster
TOPICS='^(/uav[0-9]+)?/livox/(lidar|imu)(_[0-9_]+)?$|^/(ouster|os_cloud_node[0-9_]*)/(points|imu)$|^/(camera|d400)/color/image_raw/compressed$'

# 仅在显式预检时要求匹配话题已出现。
if [ "${CHECK_ONLY:-0}" = "1" ]; then
    matched=$(rostopic list 2>/dev/null | grep -E "$TOPICS")
    if [ -z "$matched" ]; then
        echo "没有话题匹配，检查驱动是否启动、话题是否在 /uavX 命名空间下：" >&2
        rostopic list 2>/dev/null | head -40 >&2
        exit 1
    fi
    echo "$matched"
    exit 0
fi

MAX_SIZE="${MAX_SIZE:-4096}"  # 分包阈值（MB）

# rosbag 会持续发现并订阅后续出现的匹配话题。
exec rosbag record \
-b 256 \
--split \
--size="$MAX_SIZE" \
--regex \
"$TOPICS"
