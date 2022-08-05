from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rplidar_ros',
            executable='rplidar_scan_publisher',
        ),
        Node(
            package='clustering_2d_brake',
            executable='scan_to_cloud',
        ),
        Node(
            package='clustering_2d_brake',
            executable='cluster_2d',
        ),
    ])