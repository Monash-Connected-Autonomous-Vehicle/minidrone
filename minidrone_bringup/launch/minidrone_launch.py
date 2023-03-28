from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

	velodyne_path = PathJoinSubstitution(
		[FindPackageShare('velodyne'), 'launch', 'velodyne-all-nodes-VLP16-launch.py']
	)
	return LaunchDescription([
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource(velodyne_path)
		)
	])

