#!/bin/bash
# Jetson Nano 专用启动脚本：关闭图形界面，释放内存
sudo systemctl stop lightdm
export ROS_DOMAIN_ID=1
export TURTLEBOT3_MODEL=burger

# 低功耗模式运行
echo performance | sudo tee /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor

# 启动核心节点（关闭调试工具，降低资源占用）
cd ~/ros2_robot_final
source install/setup.bash
ros2 launch robot_bringup all_in_one.launch.py use_sim_time:=false &
ros2 run fault_diagnosis fault_diagnosis_node &
ros2 run web_monitor web_monitor_node &

# 启动 TensorRT 加速的强化学习避障节点
ros2 run rl_avoidance rl_avoid_node_trt &
