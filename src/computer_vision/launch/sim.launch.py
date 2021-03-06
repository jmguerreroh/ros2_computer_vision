# Copyright (c) 2022 José Miguel Guerrero Hernández
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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import yaml


def generate_launch_description():

    cv_dir = get_package_share_directory('computer_vision')

    config = os.path.join(cv_dir, 'config', 'params.yaml')

    with open(config, "r") as stream:
        try:
            conf = (yaml.safe_load(stream))

        except yaml.YAMLError as exc:
            print(exc)

    tiago_sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(cv_dir, 'launch', 'tiago_cv.launch.py')),
        launch_arguments={
          'world_name': conf['computer_vision']['world']
        }.items())
        
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    ld = LaunchDescription()

    ld.add_action(tiago_sim_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    return ld
