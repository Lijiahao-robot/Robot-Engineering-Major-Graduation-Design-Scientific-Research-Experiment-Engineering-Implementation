from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rtabmap_ros',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'subscribe_depth': True,
                'subscribe_rgb': False,
                'subscribe_scan': True,  # 仅用激光雷达，关闭视觉以降算力
                'grid_size': 0.05,
                'max_memory': 1000,  # 限制内存占用
                'odom_slam': True,
                'loop_detection': True,
                'loop_detection_threshold': 0.8,
                'min_inliers': 15,  # 降低匹配内点要求，提升速度
            }],
            remappings=[
                ('scan', '/scan'),
                ('odom', '/odom'),
                ('rtabmap/map', '/map')
            ]
        ),
        Node(
            package='rtabmap_ros',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=[{
                'frame_id': 'base_link'
            }]
        )
    ])
