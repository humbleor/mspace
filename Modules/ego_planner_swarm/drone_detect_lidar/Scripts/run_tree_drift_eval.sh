#!/bin/bash
# ============================================================================
#
# 用法:
#   ./run_tree_drift_eval.sh [时长] \
#       [uav1_dx] [uav1_dy] [uav1_yaw_deg] \
#       [uav2_dx] [uav2_dy] [uav2_yaw_deg]
#
# 示例:
#   # 仅 UAV2 有漂移 (UAV1 作为 anchor)
#   ./run_tree_drift_eval.sh 120 0 0 0 0.5 0.3 5.0
#
# ============================================================================

set -e
trap 'kill 0; exit 130' INT TERM

DURATION="${1:-120}"
DRIFT1_DX="${2:-0.0}"
DRIFT1_DY="${3:-0.0}"
DRIFT1_YAW_DEG="${4:-0.0}"
DRIFT2_DX="${5:-0.5}"
DRIFT2_DY="${6:-0.3}"
DRIFT2_YAW_DEG="${7:-5.0}"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
OUTDIR="${SCRIPT_DIR}/drift_eval_$(date +%Y%m%d_%H%M%S)"

HAS_DRIFT=$(python3 -c "
d1 = abs(${DRIFT1_DX}) + abs(${DRIFT1_DY}) + abs(${DRIFT1_YAW_DEG})
d2 = abs(${DRIFT2_DX}) + abs(${DRIFT2_DY}) + abs(${DRIFT2_YAW_DEG})
print('true' if max(d1, d2) > 1e-9 else 'false')
")

if [ "${HAS_DRIFT}" != "true" ]; then
    echo "WARNING: 双方漂移均为0，评估无意义"
fi

source /opt/ros/noetic/setup.bash
source /home/mspace/workspace/mspace/devel/setup.bash 2>/dev/null || \
source "${SCRIPT_DIR}/../../../../../../devel/setup.bash" || {
    echo "ERROR: cannot source devel/setup.bash"
    exit 1
}

mkdir -p "${OUTDIR}"

# 保存配置
cat > "${OUTDIR}/eval_config.json" << EOF
{
  "duration": ${DURATION},
  "drift1_dx": ${DRIFT1_DX},
  "drift1_dy": ${DRIFT1_DY},
  "drift1_yaw_deg": ${DRIFT1_YAW_DEG},
  "drift2_dx": ${DRIFT2_DX},
  "drift2_dy": ${DRIFT2_DY},
  "drift2_yaw_deg": ${DRIFT2_YAW_DEG},
  "scenario": "tree_drift_injection"
}
EOF

# ========== 1. 启动仿真 (不含 lidar_detect 节点) ==========
echo "[1a/7] 启动仿真 (不含 tree_detect/loc 节点) ..."
roslaunch drone_detect_lidar 2uav_lidar_detect_sim.launch \
    enable_lidar_detect:=false \
    > "${OUTDIR}/launch_stdout.log" 2>&1 &
LAUNCH_PID=$!

# ========== 1b. 等待 roscore ==========
echo "[1b/7] 等待 roscore 就绪 ..."
for i in $(seq 1 30); do
    if rostopic list >/dev/null 2>&1; then
        echo "      roscore 就绪 (等待 ${i}s)"
        break
    fi
    sleep 1
done

# ========== 1c. 等待仿真 topics ==========
echo "[1c/7] 等待仿真 topics ..."
for i in $(seq 1 90); do
    if rostopic list 2>/dev/null | grep -q "/uav1/pcl_render_node/cloud" && \
       rostopic list 2>/dev/null | grep -q "/uav2/pcl_render_node/cloud" && \
       rostopic list 2>/dev/null | grep -q "/uav1/lidar_slam/odom" && \
       rostopic list 2>/dev/null | grep -q "/uav2/lidar_slam/odom"; then
        echo "      仿真 topics 就绪 (等待 ${i}s)"
        break
    fi
    if [ $i -eq 90 ]; then
        echo "ERROR: 仿真 topics 超时"
        kill ${LAUNCH_PID} 2>/dev/null || true
        exit 1
    fi
    sleep 1
done

# ========== 2. 启动 tree_detector_node × 2 ==========
echo "[2/7] 启动 tree_detector_node ..."
PARAMS_FILE="$(rospack find drone_detect_lidar)/config/params.yaml"

for uid in 1 2; do
    rosrun drone_detect_lidar tree_detector_node \
        _drone_id:=${uid} \
        _frame_id:=world \
        ~lidar_cloud:=/uav${uid}/pcl_render_node/cloud \
        ~odom:=/uav${uid}/lidar_slam/odom \
        ~tree_detection:=/uav${uid}/tree_detection \
        ~tree_cloud:=/uav${uid}/tree_cloud \
        __ns:=uav${uid} \
        __name:=tree_detector \
        > "${OUTDIR}/tree_detector_uav${uid}.log" 2>&1 &
    # 加载检测参数 (patchwork, cluster, PCA 阈值等)
    rosparam load "${PARAMS_FILE}" /uav${uid}/tree_detector
done
echo "      tree_detector_node × 2 已启动"

# ========== 3. 启动 tree_drift_injector × 2 ==========
echo "[3/7] 启动 tree_drift_injector ..."
python3 "${SCRIPT_DIR}/tree_drift_injector.py" \
    _drone_id:=1 \
    _drift_dx:=${DRIFT1_DX} \
    _drift_dy:=${DRIFT1_DY} \
    _drift_dyaw_deg:=${DRIFT1_YAW_DEG} \
    __name:=tree_drift_injector_uav1 \
    > "${OUTDIR}/drift_injector_uav1.log" 2>&1 &

python3 "${SCRIPT_DIR}/tree_drift_injector.py" \
    _drone_id:=2 \
    _drift_dx:=${DRIFT2_DX} \
    _drift_dy:=${DRIFT2_DY} \
    _drift_dyaw_deg:=${DRIFT2_YAW_DEG} \
    __name:=tree_drift_injector_uav2 \
    > "${OUTDIR}/drift_injector_uav2.log" 2>&1 &
echo "      tree_drift_injector × 2 已启动"

# ========== 4. 创建 topic 中继 ==========
echo "[4/7] 创建 topic 中继 ..."

# 里程计中继 (供 tree_loc_node 用)
rosrun topic_tools relay /uav2/lidar_slam/odom /uav1/neighbor_odom \
    __name:=odom_relay_uav2_to_uav1 \
    > "${OUTDIR}/odom_relay_2to1.log" 2>&1 &
rosrun topic_tools relay /uav1/lidar_slam/odom /uav2/neighbor_odom \
    __name:=odom_relay_uav1_to_uav2 \
    > "${OUTDIR}/odom_relay_1to2.log" 2>&1 &

# 漂移后树检测中继
rosrun topic_tools relay /uav2/tree_detection_drifted /uav1/neighbor_tree_detection \
    __name:=tree_relay_uav2_to_uav1 \
    > "${OUTDIR}/tree_relay_2to1.log" 2>&1 &
rosrun topic_tools relay /uav1/tree_detection_drifted /uav2/neighbor_tree_detection \
    __name:=tree_relay_uav1_to_uav2 \
    > "${OUTDIR}/tree_relay_1to2.log" 2>&1 &
echo "      topic relays 已创建"

# ========== 5. 等待漂移后树检测 topics 就绪 ==========
echo "[5a/7] 等待 drifted tree topics 就绪 ..."
for i in $(seq 1 60); do
    if rostopic list 2>/dev/null | grep -q "/uav1/tree_detection_drifted" && \
       rostopic list 2>/dev/null | grep -q "/uav2/tree_detection_drifted" && \
       rostopic list 2>/dev/null | grep -q "/uav1/neighbor_tree_detection" && \
       rostopic list 2>/dev/null | grep -q "/uav2/neighbor_tree_detection"; then
        echo "      drifted tree topics 就绪 (等待 ${i}s)"
        break
    fi
    if [ $i -eq 60 ]; then
        echo "ERROR: drifted tree topics 超时"
        kill ${LAUNCH_PID} 2>/dev/null || true
        exit 1
    fi
    sleep 1
done

# ========== 5b. 启动 tree_loc_node × 2 ==========
echo "[5b/7] 启动 tree_loc_node ..."
for uid in 1 2; do
    rosrun drone_detect_lidar tree_loc_node \
        _drone_id:=${uid} \
        ~self_tree_detection:=/uav${uid}/tree_detection_drifted \
        ~neighbor_tree_detection:=/uav${uid}/neighbor_tree_detection \
        ~self_odom:=/uav${uid}/lidar_slam/odom \
        ~neighbor_odom:=/uav${uid}/neighbor_odom \
        ~tree_pose_error:=/uav${uid}/tree_pose_error \
        ~tree_relative_pose:=/uav${uid}/tree_relative_pose \
        ~tree_relative_pose_msg:=/uav${uid}/tree_relative_pose_msg \
        ~matched_tree_pairs:=/uav${uid}/matched_tree_pairs \
        ~tri_descriptors:=/uav${uid}/tri_descriptors \
        __ns:=uav${uid} \
        __name:=tree_loc \
        > "${OUTDIR}/tree_loc_uav${uid}.log" 2>&1 &
    # 加载匹配参数 (triangle, hash, SVD 质量阈值等)
    rosparam load "${PARAMS_FILE}" /uav${uid}/tree_loc
done
echo "      tree_loc_node × 2 已启动"

# ========== 5c. 等待 tree_pose_error ==========
echo "[5c/7] 等待 tree_pose_error topics ..."
for i in $(seq 1 60); do
    if rostopic list 2>/dev/null | grep -q "/uav1/tree_pose_error" && \
       rostopic list 2>/dev/null | grep -q "/uav2/tree_pose_error"; then
        echo "      tree_pose_error topics 就绪 (等待 ${i}s)"
        break
    fi
    if [ $i -eq 60 ]; then
        echo "WARNING: tree_pose_error topics 超时。树匹配可能未产生有效输出。"
        echo "检查树检测是否正常: rostopic echo /uav1/tree_detection"
    fi
    sleep 1
done

# ========== 6. 录制 rosbag ==========
echo "[6/7] 录制 rosbag ${DURATION}s ..."
rosbag record -O "${OUTDIR}/drift_eval.bag" \
    /uav1/tree_pose_error \
    /uav2/tree_pose_error \
    /uav1/tree_relative_pose_msg \
    /uav2/tree_relative_pose_msg \
    /uav1/lidar_slam/odom \
    /uav2/lidar_slam/odom \
    /uav1/tree_detection \
    /uav2/tree_detection \
    /uav1/tree_detection_drifted \
    /uav2/tree_detection_drifted \
    __name:=bag_recorder \
    > "${OUTDIR}/rosbag_stdout.log" 2>&1 &
BAG_PID=$!

sleep ${DURATION}

# ========== 7. 清理 ==========
echo "[7/7] 清理进程 + 评估 ..."
rosnode kill /bag_recorder 2>/dev/null || kill ${BAG_PID} 2>/dev/null || true
sleep 1
kill ${LAUNCH_PID} 2>/dev/null || true
pkill -f "tree_drift_injector" 2>/dev/null || true
wait ${LAUNCH_PID} 2>/dev/null || true

# ========== 8. 评估 ==========
echo ""
echo "====== 方案B: 漂移估计精度评估 ======"
python3 "${SCRIPT_DIR}/evaluate_drift_estimation.py" "${OUTDIR}"

echo ""
echo "====== 方案C: 修正效果评估 — 模拟 swarm_controller 修正 ======"
echo "  (离线注入漂移 → 应用修正公式 → 对比 |corrected - gt| vs |raw - gt|)"
python3 "${SCRIPT_DIR}/evaluate_correction_offline.py" "${OUTDIR}" 2>/dev/null || \
    echo "  (方案C 评估失败 — 检查 tree_pose_error 和 odom 数据)"

echo ""
echo "====== 完成 ======"
echo "输出目录: ${OUTDIR}/"
echo " 方案B (漂移估计):   drift_est_report.pdf / summary.json / per_frame.csv"
echo " 方案C (修正效果):   correction_offline_report.pdf / summary.json / per_frame.csv"
