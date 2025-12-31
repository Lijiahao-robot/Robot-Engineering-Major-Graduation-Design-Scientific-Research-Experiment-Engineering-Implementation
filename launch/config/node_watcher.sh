#!/bin/bash
# ROS2 节点守护脚本 - 自动重启崩溃的核心节点
# 日志文件路径
LOG_FILE="./node_watcher.log"

# 要监控的节点列表
NODES=("pid_velocity_node" "vla_avoidance_node" "pose_fusion_node" "sensor_fusion_avoid_node")

# 日志函数
log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" | tee -a $LOG_FILE
}

# 检查节点状态函数
check_and_restart() {
    local node_name=$1
    if ! ros2 node list | grep -q "$node_name"; then
        log "WARNING: $node_name 已崩溃，正在重启..."
        case $node_name in
            "pid_velocity_node")
                ros2 run pid_velocity_controller pid_velocity_node &
                ;;
            "vla_avoidance_node")
                ros2 run vla_local_planner vla_avoidance_node &
                ;;
            "pose_fusion_node")
                ros2 run robot_bringup pose_fusion_node &
                ;;
            "sensor_fusion_avoid_node")
                ros2 run robot_bringup sensor_fusion_avoid_node &
                ;;
        esac
        log "INFO: $node_name 重启完成"
    fi
}

# 初始化日志
log "INFO: 节点守护进程启动 - 监控节点: ${NODES[*]}"

# 循环监控
while true; do
    for node in "${NODES[@]}"; do
        check_and_restart "$node"
    done
    sleep 2
done
