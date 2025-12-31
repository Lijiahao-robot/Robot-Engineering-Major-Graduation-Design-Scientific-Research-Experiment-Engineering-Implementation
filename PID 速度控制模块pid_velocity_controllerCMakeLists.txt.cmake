cmake_minimum_required(VERSION 3.8)
project(pid_velocity_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# 查找依赖
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp_logging_spdlog REQUIRED)

# 编译可执行文件
add_executable(pid_velocity_node src/pid_velocity_controller.cpp)
ament_target_dependencies(pid_velocity_node 
  rclcpp 
  geometry_msgs 
  nav_msgs 
  rclcpp_logging_spdlog
)

# 安装可执行文件
install(TARGETS
  pid_velocity_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
