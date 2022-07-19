#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Darby Lim, Ryan Shim

import os
import math

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file_name = 'waffle.model'
    world = os.path.join(get_package_share_directory('my_turtlebot'),
                         'worlds', world_file_name)
    launch_file_dir = os.path.join(get_package_share_directory('my_turtlebot'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_teleop= get_package_share_directory('turtlebot3_teleop')
    pkg_octomap = get_package_share_directory('octomap_server2')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        Node(package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=["0", "0", "-0.45", "0", "0", str(-math.pi/2), 'map', 'odom']),
            #arguments=["0", "0", "0", "0", "0", "0", 'map', 'odom']),

        Node(package='octomap_server2',
            executable='octomap_server',
            output='screen',
            remappings=[('cloud_in', '/camera/points')],
            parameters=[{'resolution': 0.1,
              'frame_id': 'map',
              'base_frame_id': 'base_footprint',
              'height_map': True,
              'colored_map': True,
              'color_factor': 0.8,
              'filter_ground': False,
              'filter_speckles': True,
              'ground_filter/distance': 0.2,
              'ground_filter/angle': 0.15,
              'ground_filter/plane_distance': 0.0,
              'compress_map': False,
              'incremental_2D_projection': True,
              'sensor_model/max_range': -1.0,
              'sensor_model/hit': 0.7,
              'sensor_model/miss': 0.4,
              'sensor_model/min': 0.12,
              'sensor_model/max': 0.97,
              'color/r': 0.0,
              'color/g': 0.0,
              'color/b': 1.0,
              'color/a': 1.0,
              'color_free/r': 0.0,
              'color_free/g': 0.0,
              'color_free/b': 1.0,
              'color_free/a': 1.0,
              'publish_free_space': False,
              'pointcloud_min_z': 0.05,
              'pointcloud_max_z': 1.0,
            }]
        )
        
        
    ])