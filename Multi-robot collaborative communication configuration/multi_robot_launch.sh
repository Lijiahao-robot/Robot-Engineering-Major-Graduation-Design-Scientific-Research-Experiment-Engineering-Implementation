#!/bin/bash
# 启动 2 台机器人的仿真环境
export TURTLEBOT3_MODEL=burger

# 机器人 1：命名空间 robot1
ros2 launch turtlebot3_gazebo multi_turtlebot3.launch.py num_robots:=2 &
sleep 10

# 机器人 1 启动 SLAM+避障
ros2 launch robot_bringup all_in_one.launch.py __ns:=/robot1 use_sim_time:=true &
# 机器人 2 启动 SLAM+避障
ros2 launch robot_bringup all_in_one.launch.py __ns:=/robot2 use_sim_time:=true &

# 启动多机器人 RViz 可视化
ros2 run rviz2 rviz2 -d src/robot_bringup/config/multi_robot_view.rviz
