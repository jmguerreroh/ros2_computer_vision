# Copyright (c) 2021 José Miguel Guerrero Hernández
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    
    tiago_gazebo_dir = get_package_share_directory('tiago_gazebo')
    tiago_sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tiago_gazebo_dir, 'launch', 'tiago_gazebo.launch.py')),
        launch_arguments={
          'world_name': 'home'
        }.items())

    #tiago_sim_cmd = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(os.path.join(tiago_gazebo_dir, 'launch', 'tiago_gazebo.launch.py')),
    #    )
    
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    ld = LaunchDescription()


    ld.add_action(tiago_sim_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    #hospital_pkg_dir = get_package_share_directory('aws_robomaker_hospital_world')
    #hospital_launch_path = os.path.join(hospital_pkg_dir, 'launch')
    #hospital_world_cmd = IncludeLaunchDescription(
    #        PythonLaunchDescriptionSource([hospital_launch_path, '/hospital.launch.py'])
    #    )
    #ld.add_action(hospital_world_cmd)

    return ld
