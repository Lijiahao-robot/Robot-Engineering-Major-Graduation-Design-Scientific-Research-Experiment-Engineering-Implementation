#!/bin/bash
# 机器人工程完整代码包一键启动脚本
# 日志文件
RUN_LOG="./run_all.log"

# 日志函数
log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" | tee -a $RUN_LOG
}

# 检查 ROS2 环境
if [ -z "$ROS_DISTRO" ]; then
    log "ERROR: 未检测到 ROS2 环境，请先执行 source /opt/ros/humble/setup.bash"
    exit 1
fi

# 进入工作空间
cd ~/ros2_robot_final
source install/setup.bash
log "INFO: 已进入 ROS2 工作空间并加载环境变量"

# 启动仿真环境（真实机器人请注释以下两行）
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py &
log "INFO: 已启动 Gazebo 仿真环境"
sleep 5

# 启动节点守护进程
./node_watcher.sh &
log "INFO: 已启动节点守护进程"
sleep 2

# 启动 SLAM+VLM 融合定位
ros2 launch robot_bringup slam_vlm_fusion.launch.py &
log "INFO: 已启动 SLAM+VLM 融合定位"
sleep 3

# 启动避障+控制+多传感器融合
ros2 launch robot_bringup all_in_one.launch.py &
log "INFO: 已启动避障+控制+多传感器融合"
sleep 2

# 启动调试工具
ros2 run rqt_plot rqt_plot /fused_odom/pose/pose/position/x /fused_odom/pose/pose/position/y &
ros2 run rqt_reconfigure rqt_reconfigure &
log "INFO: 已启动 RQT 调试工具"

# 启动 RViz 可视化
ros2 run rviz2 rviz2 -d src/robot_bringup/config/robot_view.rviz
log "INFO: 已启动 RViz2 可视化"

# 等待所有进程结束
wait
