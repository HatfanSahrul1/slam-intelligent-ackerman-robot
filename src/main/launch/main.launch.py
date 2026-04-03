from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    laser_frame = LaunchConfiguration('laser_frame')
    stm32_port = LaunchConfiguration('stm32_port')
    arduino_port = LaunchConfiguration('arduino_port')

    # optional: pose lidar relatif ke base_footprint
    lidar_x = LaunchConfiguration('lidar_x')
    lidar_y = LaunchConfiguration('lidar_y')
    lidar_z = LaunchConfiguration('lidar_z')
    lidar_roll = LaunchConfiguration('lidar_roll')
    lidar_pitch = LaunchConfiguration('lidar_pitch')
    lidar_yaw = LaunchConfiguration('lidar_yaw')

    return LaunchDescription([
        # ---- Args ----
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('serial_baudrate', default_value='460800'),  # RPLIDAR C3 kamu
        DeclareLaunchArgument('laser_frame', default_value='laser_frame'),

        DeclareLaunchArgument('stm32_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('arduino_port', default_value='/dev/ttyACM0'),

        # TF lidar offset (default 0 semua)
        DeclareLaunchArgument('lidar_x', default_value='0'),
        DeclareLaunchArgument('lidar_y', default_value='0'),
        DeclareLaunchArgument('lidar_z', default_value='0'),
        DeclareLaunchArgument('lidar_roll', default_value='0'),
        DeclareLaunchArgument('lidar_pitch', default_value='0'),
        DeclareLaunchArgument('lidar_yaw', default_value='0'),

        # ---- RPLIDAR driver ----
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate,
                'frame_id': laser_frame,
                'inverted': False,
                'angle_compensate': True,
            }],
        ),

        # ---- Nodes kamu ----
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
            output='screen',
            parameters=[{
                'stm32_port': stm32_port,
                'arduino_port': arduino_port,
            }],
        ),
        Node(
            package='simple_bridge',
            executable='scan_right',
            name='scan_right',
            output='screen'
        ),

        # ---- Static TF: base_footprint -> laser_frame (new-style args) ----
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_laser',
            arguments=[
                '--x', lidar_x,
                '--y', lidar_y,
                '--z', lidar_z,
                '--roll', lidar_roll,
                '--pitch', lidar_pitch,
                '--yaw', lidar_yaw,
                '--frame-id', 'base_footprint',
                '--child-frame-id', laser_frame,
            ],
            output='screen',
        ),
    ])