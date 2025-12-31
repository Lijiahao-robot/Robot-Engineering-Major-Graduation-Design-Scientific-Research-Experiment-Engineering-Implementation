import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # 声明仿真时间参数
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        # 1. PID 速度控制节点
        Node(
            package='pid_velocity_controller',
            executable='pid_velocity_node',
            name='pid_velocity_node',
            output='screen',
            parameters=[
                {'kp': 0.5},
                {'ki': 0.1},
                {'kd': 0.05},
                {'target_vel': 0.2},
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ]
        ),

        # 2. VLA 局部避障节点
        Node(
            package='vla_local_planner',
            executable='vla_avoidance_node',
            name='vla_avoidance_node',
            output='screen',
            parameters=[
                {'safe_radius': 0.5},
                {'sector_num': 36},
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ]
        ),

        # 3. 激光-视觉融合避障节点
        Node(
            package='robot_bringup',
            executable='sensor_fusion_avoid_node.py',
            name='sensor_fusion_avoid_node',
            output='screen',
            parameters=[
                {'safe_radius': 0.6},
                {'use_camera': True},
                {'buffer_size': 5},
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ]
        ),

        # 4. RealSense 深度相机驱动（真实硬件）
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense2_camera_node',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        # 5. 激光雷达驱动（RPLIDAR）
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            output='screen',
            parameters=[
                {'serial_port': '/dev/ttyUSB0'},
                {'serial_baudrate': 115200},
                {'frame_id': 'laser'},
                {'inverted': False},
                {'angle_compensate': True},
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ]
        )
    ])
