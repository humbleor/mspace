#!/bin/bash
# 全量飞行包：显式话题清单，不用 -a。
#   CHECK_ONLY=1 ./topiclist_more.sh   只检查清单里每个话题当前是否存在，不写文件
#   ./topiclist_more.sh                录制，Ctrl+C 结束
# 环境变量：BAG_DIR PREFIX MAX_SIZE(MB) MIN_SPACE LIDAR_ID WITH_CLOUDS(0=不录点云)

BAG_DIR="${BAG_DIR:-$HOME/data}"
PREFIX="${PREFIX:-flight_$(date +%Y%m%d-%H%M%S)}"
MAX_SIZE="${MAX_SIZE:-4096}"      # 单文件上限 MB
MIN_SPACE="${MIN_SPACE:-5G}"      # 剩余空间低于此值自动停
LIDAR_ID="${LIDAR_ID:-192_168_2_174}"
WITH_CLOUDS="${WITH_CLOUDS:-1}"
NS=/uav1                          # 多机改 2 / 循环

CORE=(
  /tf /tf_static                                         # 回放必需
  $NS/livox/lidar_$LIDAR_ID  $NS/livox/imu_$LIDAR_ID     # 雷达原始
  $NS/drone_Odom_high_freq                               # LIO 位姿
  $NS/mavros/state  $NS/mavros/local_position/odom        # 飞控状态与本地里程计
  $NS/mavros/vision_pose/pose  $NS/mavros/setpoint_raw/local
)

EXTRA=(
  $NS/drone_Odometry  $NS/drone_path                     # LIO 低频位姿/轨迹
  $NS/prometheus/drone_state  $NS/local/drone_state      # 公共状态（world 模式用后者）
  $NS/world/odom  $NS/world/setpoint_raw/local  $NS/frame_ready
  $NS/planning/bspline  $NS/planning/ego/traj_cmd  $NS/planning/swarm_trajs
  $NS/prometheus/swarm_command  $NS/prometheus/message/main  $NS/traj_start_trigger
  $NS/mavros/battery  $NS/mavros/imu/data
  /camera/color/image_raw/compressed                     # 双机脚本里是 /uav1_camera/...
)

TOPICS=("${CORE[@]}" "${EXTRA[@]}")
[ "$WITH_CLOUDS" = "1" ] && TOPICS+=("$NS/drone_cloud_registered" "$NS/world/cloud")

mkdir -p "$BAG_DIR" && cd "$BAG_DIR" || exit 1

# ---- 预检：清单里哪些话题现在存在 ----
published="$(rostopic list 2>/dev/null)"
missing_core=0
echo "== 话题预检 =="
for t in "${TOPICS[@]}"; do
  if grep -Fxq "$t" <<<"$published"; then
    echo "  ok      $t"
  else
    echo "  MISSING $t"
    for c in "${CORE[@]}"; do [ "$c" = "$t" ] && missing_core=$((missing_core+1)); done
  fi
done
if [ "$missing_core" -gt 0 ]; then
  echo "核心话题缺失 $missing_core 个：检查 NS、LIDAR_ID、驱动是否启动" >&2
  exit 1
fi
# 未发布但稍后才出现的话题（如 world/odom、traj）不影响：rosbag 按名字订阅，出现后自动录
[ "${CHECK_ONLY:-0}" = "1" ] && exit 0

df -h "$BAG_DIR" | tail -1
echo "录制到 $BAG_DIR/${PREFIX}*.bag（Ctrl+C 结束）"
rosbag record "${TOPICS[@]}" \
  -b 256 \
  --chunksize=1024 \
  --lz4 \
  --split --size="$MAX_SIZE" \
  -L "$MIN_SPACE" \
  --tcpnodelay \
  -O "$PREFIX"