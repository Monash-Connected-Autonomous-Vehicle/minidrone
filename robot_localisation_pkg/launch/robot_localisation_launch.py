# Copyright (c) 2021 Juan Miguel Jimeno
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = True

    ekf_gps_config_path = PathJoinSubstitution(
        [FindPackageShare("robot_localisation_pkg"),"config", "ekf_gps.yaml"]
    )

    ekf_odom_config_path = PathJoinSubstitution(
        [FindPackageShare("robot_localisation_pkg"),"config", "ekf_odom.yaml"]
    )

    ekf_pos_config_path = PathJoinSubstitution(
        [FindPackageShare("robot_localisation_pkg"),"config", "ekf_pos.yaml"]
    )

    return LaunchDescription([
        # Node( 
        #     package='tf2_ros',
        #     node_executable='static_transform_publisher',
        #     node_name='static_transform1',
        #     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', '1.0', 'map', 'odom'],
    	#     parameters=[{'use_sim_time': use_sim_time}]
        # ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_positioning_filter_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                ekf_pos_config_path
            ],
            remappings=[("odometry/filtered", "odometry/filtered_map")]
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_odometry_filter_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                ekf_odom_config_path
            ]
        ),

        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                ekf_gps_config_path
            ],
            remappings=[("odometry/filtered", "odometry/filtered_map"),("/gps/fix", "/gps"),("/imu", "/imu/data")], 
        )
        
    ])

# sources:
#https://navigation.ros.org/setup_guides/index.html#
# https://answers.ros.org/question/374976/ros2-launch-gazebolaunchpy-from-my-own-launch-file/
# https://github.com/ros2/rclcpp/issues/940


# START
# x: 1.379847526550293e-05
#       y: -0.0003164033405482769
#       z: 0.0

# # MOVE IN POSITIVE X
# x: -1.0165120549499989
# y: -4.949262981070206
# z: 0.0

# # MOVE IN POSITIVE Y (with a little x)
#  x: 7.039857481606305
#       y: -4.549564794637263
#       z: 0.0
