from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='simple_bridge',
            executable='simple_bridge',
            name='simple_bridge',
            output='screen'
        ),
        Node(
            package='simple_bridge',
            executable='scan_right',
            name='scan_right',
            output='screen'
        )
    ])