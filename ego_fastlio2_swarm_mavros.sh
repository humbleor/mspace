#!/bin/bash

set -e

export DISABLE_ROS1_EOL_WARNINGS=1

source /opt/ros/noetic/setup.bash
source ~/workspace/ws_livox/devel/setup.bash
source ~/workspace/mspace/devel/setup.bash

# 集群配置
UAV1_IP="192.168.1.50"  # joey
UAV2_IP="192.168.1.40"  # mspace
GCS_IP="192.168.1.70"  # gcs
UAV1_HOSTNAME="joey"
UAV2_HOSTNAME="mspace"
PASSWORD="123"
ROS_MASTER_URI="http://${UAV1_IP}:11311"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RESET='\033[0m'
# 日志函数
log_info() {
    echo -e "${GREEN}[INFO]${RESET} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${RESET} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${RESET} $1"
}

ROSCORE_PID=""
UAV1_PID=""
UAV2_PID=""

cleanup() {
    # 杀死UAV2的SSH进程
    if [ ! -z "$UAV2_PID" ] && kill -0 $UAV2_PID 2>/dev/null; then
        log_info "停止UAV2进程 (PID: $UAV2_PID)"
        kill $UAV2_PID 2>/dev/null || true
        # 同时杀死远程的roslaunch进程
        sshpass -p "$PASSWORD" ssh -o StrictHostKeyChecking=no ${UAV2_HOSTNAME}@${UAV2_IP} \
            "pkill -f 'roslaunch mavros_bringup px4_mavros_swarm_basic.launch'" 2>/dev/null || true
    fi
    
    # 杀死UAV1的MAVROS进程
    if [ ! -z "$UAV1_PID" ] && kill -0 $UAV1_PID 2>/dev/null; then
        log_info "停止UAV1 MAVROS进程 (PID: $UAV1_PID)"
        kill $UAV1_PID 2>/dev/null || true
    fi
    
    # 杀死roscore进程（如果是我们启动的）
    if [ ! -z "$ROSCORE_PID" ] && kill -0 $ROSCORE_PID 2>/dev/null; then
        log_info "停止roscore进程 (PID: $ROSCORE_PID)"
        kill $ROSCORE_PID 2>/dev/null || true
    fi
    
    # 杀死所有相关的roslaunch进程
    pkill -f "roslaunch mavros_bringup px4_mavros_swarm_basic.launch" 2>/dev/null || true
    
    exit 0
}

trap cleanup SIGINT SIGTERM EXIT

# 设置ROS环境
setup_ros_env() {
    local host_ip=$1
    local is_master=$2
    
    log_info "设置ROS环境: 主机 $host_ip, Master: $is_master"
    
    if [ "$is_master" = "true" ]; then
        export ROS_MASTER_URI="${ROS_MASTER_URI}"
        export ROS_IP="$host_ip"
        export ROS_HOSTNAME="$host_ip"
    else
        export ROS_MASTER_URI="${ROS_MASTER_URI}"
        export ROS_IP="$host_ip"
        export ROS_HOSTNAME="$host_ip"
    fi
    
    # 检查ROS环境
    if [ -z "$ROS_DISTRO" ]; then
        log_error "ROS环境未设置，请先source ROS"
        return 1
    fi
    
    log_info "ROS_MASTER_URI: $ROS_MASTER_URI"
    log_info "ROS_IP: $ROS_IP"
}

# 启动主机1 (ROS Master)
start_uav1() {
    log_info "启动主机1 (UAV1) - ROS Master"
    
    # 设置环境
    setup_ros_env $UAV1_IP true
    
    # 检查roscore是否运行
    if ! rostopic list > /dev/null 2>&1; then
        log_info "启动roscore..."
        roscore &
        ROSCORE_PID=$!
        sleep 3
    else
        log_info "roscore已在运行"
        ROSCORE_PID=""
    fi
    
    # 启动MAVROS
    log_info "启动UAV1的MAVROS..."
    roslaunch mavros_bringup px4_mavros_swarm_basic.launch uav_id:=1 gcs_ip:=${GCS_IP} &
    UAV1_PID=$!
    
    log_info "UAV1启动完成 (PID: $UAV1_PID)"
}

# 启动主机2 (ROS Client)
start_uav2() {
    log_info "启动主机2 (UAV2) - ROS Client"
    
    # 设置环境
    setup_ros_env $UAV2_IP false
    
    # 等待ROS Master就绪
    log_info "等待ROS Master就绪..."
    until rostopic list > /dev/null 2>&1; do
        log_warn "等待ROS Master..."
        sleep 2
    done
    
    # 通过SSH在主机2上启动MAVROS
    log_info "在主机2上启动MAVROS..."
    sshpass -p "$PASSWORD" ssh -o StrictHostKeyChecking=no ${UAV2_HOSTNAME}@${UAV2_IP} "
        set -e
        source /opt/ros/noetic/setup.bash
        source ~/workspace/ws_livox/devel/setup.bash
        source ~/workspace/mspace/devel/setup.bash

        export ROS_MASTER_URI=\"${ROS_MASTER_URI}\"
        export ROS_IP=\"${UAV2_IP}\"
        until rostopic list > /dev/null 2>&1; do
            echo '等待ROS Master...'
            sleep 2
        done
        roslaunch mavros_bringup px4_mavros_swarm_basic.launch uav_id:=2 gcs_ip:=${GCS_IP}" &
    UAV2_PID=$!
    
    log_info "UAV2启动命令已发送 (PID: $UAV2_PID)"
}

main() {
    log_info "开始启动无人机集群..."
    
    # 启动UAV1
    start_uav1
    
    # 等待UAV1完全启动
    sleep 8
    
    # 启动UAV2
    start_uav2
    
    log_info "所有无人机启动命令已发送"
    log_info "按 Ctrl+C 停止所有进程"
    
    # 等待所有后台进程
    wait
}

main

# gnome-terminal \
# --window -e 'bash -c "roslaunch mavros_bringup px4_mavros_swarm.launch; exec bash"' \
# --tab -e 'bash -c "sleep 5; roslaunch livox_ros_driver2 msg_MID360.launch; exec bash"' \
# --tab -e 'bash -c "sleep 15; roslaunch fast_lio mapping_mid360.launch; exec bash"' \
# --tab -e 'bash -c "sleep 10; roslaunch prometheus_swarm_control ego_swarm_control.launch; exec bash"' \
# --tab -e 'bash -c "sleep 10; roslaunch ego_planner osfastlio2_simdemo.launch; exec bash"' \
# --tab -e 'bash -c "sleep 15; roslaunch prometheus_swarm_control ego_station.launch; exec bash"' \
# --tab -e 'bash -c "sleep 5; roslaunch realsense2_camera rs_camera.launch; exec bash"'

