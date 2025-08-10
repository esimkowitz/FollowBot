from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_foxglove = LaunchConfiguration('use_foxglove')
    rplidar_port = LaunchConfiguration('rplidar_port')
    rplidar_frame = LaunchConfiguration('rplidar_frame')
    params_file = LaunchConfiguration('params_file')
    bringup_share = get_package_share_directory('bringup')
    default_params = os.path.join(bringup_share, 'config', 'nav2_params.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('use_foxglove', default_value='false'),
        DeclareLaunchArgument('rplidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('rplidar_frame', default_value='laser'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        Node(package='rplidar_ros', executable='rplidar_composition', name='rplidar',
             parameters=[{'serial_port': rplidar_port, 'serial_baudrate': 115200, 'frame_id': rplidar_frame, 'angle_compensate': True}]),
        Node(package='rvr_ros', executable='rvr_ros_node', name='rvr_ros',
             parameters=[{'port': '/dev/ttyAMA0', 'baud': 115200, 'deadman': 0.2}]),
        Node(package='leg_detector', executable='leg_detector_node', name='leg_detector',
             parameters=[{'jump_thresh': 0.2, 'min_pts': 3, 'max_pts': 25, 'min_r': 0.05, 'max_r': 0.12}]),
        Node(package='tracker_fuser', executable='tracker_fuser_node', name='tracker_fuser',
             parameters=[{'standoff': 1.2, 'reissue_period': 1.5, 'frame_id': 'base_link'}]),
        Node(package='nav2_controller', executable='controller_server', output='screen', parameters=[params_file]),
        Node(package='nav2_planner', executable='planner_server', output='screen', parameters=[params_file]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', output='screen', parameters=[params_file]),
        Node(package='nav2_behavior_tree', executable='behavior_server', output='screen', parameters=[params_file]),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower', output='screen', parameters=[params_file]),
        Node(package='foxglove_bridge', executable='foxglove_bridge', name='foxglove_bridge',
             condition=(use_foxglove == 'true'),
             parameters=[{'port': 8765, 'address': '0.0.0.0', 'client_publish': True}]),
    ])
