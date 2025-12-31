cmake_minimum_required(VERSION 3.8)
project(robot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# 查找依赖
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp_logging_spdlog REQUIRED)

# 包含头文件
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# 编译卡尔曼融合节点
add_executable(pose_fusion_node src/pose_fusion_node.cpp)
ament_target_dependencies(pose_fusion_node 
  rclcpp 
  nav_msgs 
  tf2_ros 
  rclcpp_logging_spdlog
)
target_link_libraries(pose_fusion_node ${EIGEN3_LIBRARIES})

# 安装可执行文件
install(TARGETS
  pose_fusion_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装启动文件和配置文件
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Python 模块配置
ament_python_install_package(${PROJECT_NAME})

# 安装 Python 节点
install(PROGRAMS
  src/sensor_fusion_avoid_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
