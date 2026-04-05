from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    stm32_port = LaunchConfiguration('stm32_port')
    arduino_port = LaunchConfiguration('arduino_port')

    return LaunchDescription([
        DeclareLaunchArgument('stm32_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('arduino_port', default_value='/dev/ttyACM0'),

        Node(
            package='simple_bridge',
            executable='simple_bridge',
            name='simple_bridge',
            output='screen',
            parameters=[{
                'stm32_port': stm32_port,
                'arduino_port': arduino_port,
            }]
        ),
        Node(
            package='simple_bridge',
            executable='scan_right',
            name='scan_right',
            output='screen'
        )
    ])