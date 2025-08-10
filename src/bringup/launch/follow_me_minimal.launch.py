from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_foxglove = LaunchConfiguration('use_foxglove')
    rplidar_port = LaunchConfiguration('rplidar_port')
    rplidar_frame = LaunchConfiguration('rplidar_frame')

    return LaunchDescription([
        DeclareLaunchArgument('use_foxglove', default_value='false'),
        DeclareLaunchArgument('rplidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('rplidar_frame', default_value='laser'),

        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar',
            parameters=[{
                'serial_port': rplidar_port,
                'serial_baudrate': 115200,
                'frame_id': rplidar_frame,
                'angle_compensate': True
            }]
        ),

        Node(
            package='rvr_ros',
            executable='rvr_ros_node',
            name='rvr_ros',
            parameters=[{'port': '/dev/ttyAMA0', 'baud': 115200, 'deadman': 0.2}]
        ),

        Node(
            package='leg_detector',
            executable='leg_detector_node',
            name='leg_detector',
            parameters=[{'jump_thresh': 0.2, 'min_pts': 3, 'max_pts': 25, 'min_r': 0.05, 'max_r': 0.12}]
        ),

        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            condition=(use_foxglove == 'true'),
            parameters=[{'port': 8765, 'address': '0.0.0.0', 'client_publish': True}]
        ),
    ])
