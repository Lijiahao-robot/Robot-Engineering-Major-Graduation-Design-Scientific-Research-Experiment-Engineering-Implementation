import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取配置文件路径
    config_dir = os.path.join(get_package_share_directory('robot_bringup'), 'config')

    return LaunchDescription([
        # 声明仿真时间参数
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        # 1. Cartographer SLAM 节点
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', 'turtlebot3_lds_2d.lua'
            ],
            remappings=[
                ('scan', '/scan'),
                ('odom', '/odom')
            ]
        ),

        # 2. 栅格地图发布节点
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            arguments=['-resolution', '0.05']
        ),

        # 3. VLM 定位节点
        Node(
            package='vlm_localization',
            executable='vlm_localization_node',
            name='vlm_localization_node',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        # 4. 卡尔曼滤波融合节点
        Node(
            package='robot_bringup',
            executable='pose_fusion_node',
            name='pose_fusion_node',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )
    ])
