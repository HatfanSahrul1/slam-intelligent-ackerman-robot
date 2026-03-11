from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='intelligent',
            executable='intelligent_node',
            name='intelligent_node',
            output='screen',
            parameters=[]
        )
    ])