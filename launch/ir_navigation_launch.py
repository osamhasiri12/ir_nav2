from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ir_nav',
            executable='ir_navigation',
            name='ir_navigation',
            output='screen'
        )
    ])