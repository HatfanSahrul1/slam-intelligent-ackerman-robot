from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='intelligent',
            executable='intelligent_node',
            name='intelligent_node',
            output='screen'
        ),
        Node(
            package='simple_bridge',
            executable='simple_bridge',
            name='simple_bridge_node',
            output='screen'
        ),
        Node(
            package='simple_bridge',
            executable='scan_right',
            name='scan_right',
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'laser_frame'],
            name='static_tf_laser'
        )
    ])