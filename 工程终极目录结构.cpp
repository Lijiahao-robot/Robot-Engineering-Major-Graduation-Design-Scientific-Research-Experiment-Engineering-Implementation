ros2_robot_final/
├── src/
│   ├── pid_velocity_controller/       # PID 速度控制（C++，积分分离+看门狗）
│   │   ├── include/pid_velocity_controller/
│   │   ├── src/
│   │   │   └── pid_velocity_controller.cpp
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── vla_local_planner/             # VLA 局部避障（C++）
│   ├── vlm_localization/              # VLM 矢量场定位（Python）
│   ├── rl_avoidance/                  # 强化学习避障（Python，可选模块）
│   └── robot_bringup/                 # 启动配置&核心融合节点
│       ├── launch/
│       │   ├── all_in_one.launch.py   # 全模块启动
│       │   └── slam_vlm_fusion.launch.py  # SLAM+VLM 融合启动
│       ├── config/
│       │   ├── robot_view.rviz        # RViz 可视化配置
│       │   └── turtlebot3_lds_2d.lua  # Cartographer 配置
│       ├── src/
│       │   ├── pose_fusion_node.cpp   # 卡尔曼滤波融合节点
│       │   └── sensor_fusion_avoid_node.py  # 激光-视觉融合避障
│       ├── robot_bringup/
│       │   └── __init__.py
│       ├── package.xml
│       ├── CMakeLists.txt
│       └── setup.py
├── node_watcher.sh                    # 节点守护脚本
├── run_all.sh                         # 一键启动脚本
├── install_deps.sh                    # 依赖安装脚本
└── README.md                          # 工程说明文档
