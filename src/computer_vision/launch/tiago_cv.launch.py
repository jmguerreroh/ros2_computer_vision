# Copyright (c) 2021 PAL Robotics S.L.
# Modified by José Miguel Guerrero Hernández
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
from os import environ, pathsep

from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_pal.include_utils import include_launch_py_description

from launch_ros.actions import Node
import yaml

def get_model_paths(packages_names):
    model_paths = ""
    for package_name in packages_names:
        if model_paths != "":
            model_paths += pathsep

        package_path = get_package_prefix(package_name)
        model_path = os.path.join(package_path, "share")

        model_paths += model_path

    return model_paths


def get_resource_paths(packages_names):
    resource_paths = ""
    for package_name in packages_names:
        if resource_paths != "":
            resource_paths += pathsep

        package_path = get_package_prefix(package_name)
        resource_paths += package_path

    return resource_paths


def generate_launch_description():
    cv_dir = get_package_share_directory('computer_vision')

    config = os.path.join(cv_dir, 'config', 'params.yaml')

    with open(config, "r") as stream:
        try:
            conf = (yaml.safe_load(stream))

        except yaml.YAMLError as exc:
            print(exc)

    # Default: Original path for PAL worlds
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('pal_gazebo_worlds'),
            'launch'), '/pal_gazebo.launch.py']),
    )

    # Specific path for AWS worlds
    if "aws" in conf['computer_vision']['world']:
        # Default: Small house
        gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('aws_robomaker_small_house_world'),
                'launch'), '/view_small_house.launch.py']),
        )
        # Hospital
        if "hospital" in conf['computer_vision']['world']:
            gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('aws_robomaker_hospital_world'),
                    'launch'), '/view_hospital.launch.py']),
            )
        # Racetrack - default day. Change mode in /view_racetrack.launch.py
        if "racetrack" in conf['computer_vision']['world']:
            gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('aws_robomaker_racetrack_world'),
                    'launch'), '/view_racetrack.launch.py']),
            )
        # Small warehouse
        if "warehouse" in conf['computer_vision']['world']:
            gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('aws_robomaker_small_warehouse_world'),
                    'launch'), '/no_roof_small_warehouse.launch.py']),
            )
        # Bookstore
        if "bookstore" in conf['computer_vision']['world']:
            gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('aws_robomaker_bookstore_world'),
                    'launch'), '/view_bookstore.launch.py']),
            )


    tiago_spawn = include_launch_py_description(
        'computer_vision', ['launch', 'tiago_spawn.launch.py'])

    tiago_bringup = include_launch_py_description(
        'tiago_bringup', ['launch', 'tiago_bringup.launch.py'])

    # tuck_arm = Node(package='tiago_gazebo',
    #                 executable='tuck_arm.py',
    #                 output='both')

    # @TODO: review pal_gazebo
    # @TODO: review tiago_spawn
    # @TODO: simulation_tiago_bringup?
    # @TODO: pal_pcl_points_throttle_and_filter

    packages = ['tiago_description', 'pmb2_description',
                'hey5_description', 'pal_gripper_description']
    model_path = get_model_paths(packages)
    resource_path = get_resource_paths(packages)

    if 'GAZEBO_MODEL_PATH' in environ:
        model_path += pathsep + environ['GAZEBO_MODEL_PATH']

    if 'GAZEBO_RESOURCE_PATH' in environ:
        resource_path += pathsep + environ['GAZEBO_RESOURCE_PATH']

    return LaunchDescription([
        SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path),
        # Using this prevents shared library from being found
        SetEnvironmentVariable("GAZEBO_RESOURCE_PATH", resource_path),
        gazebo,
        TimerAction(
            period=5.0,
            actions=[tiago_spawn],
            cancel_on_shutdown=False
        ),
        TimerAction(
            period=7.0,
            actions=[tiago_bringup],
            cancel_on_shutdown=False
        )
    ])
