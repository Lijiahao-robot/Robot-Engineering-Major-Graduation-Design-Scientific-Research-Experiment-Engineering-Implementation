from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_policy',
            executable='policy_node',
            output='screen'
        )
    ])
