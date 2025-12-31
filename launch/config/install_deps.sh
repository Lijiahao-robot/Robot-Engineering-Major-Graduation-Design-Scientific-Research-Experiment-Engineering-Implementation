#!/bin/bash
# 机器人工程完整代码包依赖安装脚本
LOG_FILE="./install_deps.log"

log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" | tee -a $LOG_FILE
}

log "INFO: 开始安装所有依赖..."

# 更新系统源
sudo apt update -y | tee -a $LOG_FILE
log "INFO: 系统源已更新"

# 安装 ROS2 依赖包
sudo apt install -y \
ros-humble-cartographer ros-humble-cartographer-ros \
ros-humble-realsense2-camera ros-humble-pcl-ros \
ros-humble-turtlebot3-gazebo ros-humble-libg2o \
ros-humble-rplidar-ros ros-humble-rqt ros-humble-rqt-plot \
ros-humble-rqt-reconfigure ros-humble-tf2-tools \
ros-humble-nav2-msgs ros-humble-eigen3-cmake-module \
ros-humble-rclcpp-logging-spdlog | tee -a $LOG_FILE
log "INFO: ROS2 依赖包安装完成"

# 安装 Python 依赖
pip install pclpy numpy gymnasium stable-baselines3 transforms3d scipy | tee -a $LOG_FILE
log "INFO: Python 依赖包安装完成"

# 赋予脚本执行权限
chmod +x node_watcher.sh run_all.sh install_deps.sh | tee -a $LOG_FILE
log "INFO: 已赋予所有脚本执行权限"

log "INFO: 所有依赖安装完成！"
