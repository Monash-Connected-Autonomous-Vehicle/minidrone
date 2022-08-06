from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    lidar_share = get_package_share_directory('rplidar_ros2')

    return LaunchDescription([
        IncludeLaunchDescription([
            PythonLaunchDescriptionSource([lidar_share, '/launch/rplidar_launch.py'])
        ]),

        Node(
            package='clustering_2d_brake',
            executable='scan_to_cloud',
        ),

        Node(
            package='clustering_2d_brake',
            executable='cluster_2d',
        ),
    ])