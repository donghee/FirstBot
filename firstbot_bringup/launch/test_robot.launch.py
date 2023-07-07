# Copyright 2020 ROS2-Control Development Team (2020)
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
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # Get URDF via xacro
    robot_description_path = os.path.join(
        get_package_share_directory('diffdrive_arduino'),
        'description',
        'robot.urdf.xacro')
    robot_description_config = xacro.process_file(robot_description_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    controller_params_file = os.path.join(
        get_package_share_directory('diffdrive_arduino'),
        'controllers',
        'robot_controller.yaml'
        )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_params_file],
        #parameters=[{'robot_description': robot_description}, controller_params_file]
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["diff_cont"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    # Launch them all!
    return LaunchDescription([
        # joystick,
        #twist_mux,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
    ])

#    return LaunchDescription([
#      Node(
#        package='controller_manager',
#        executable='ros2_control_node',
#        parameters=[robot_description, controller_params_file],
#        output={
#          'stdout': 'screen',
#          'stderr': 'screen',
#          },
#        )
#
#    ])
