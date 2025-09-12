# Copyright (c) 2024 FZI Forschungszentrum Informatik
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#
# Author: Felix Exner
import os, yaml
from pathlib                     import Path
from launch                      import LaunchDescription
from launch.actions              import (IncludeLaunchDescription,
                                         OpaqueFunction, RegisterEventHandler)
from launch.conditions           import IfCondition
from launch.event_handlers       import OnProcessExit
from launch.substitutions        import (LaunchConfiguration,
                                         PathJoinSubstitution)
from launch_ros.actions          import Node
from launch_ros.substitutions    import FindPackageShare
from moveit_configs_utils        import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from aist_bringup.launch_common  import declare_launch_arguments

launch_arguments = [
    {
        'name':        'launch_servo',
        'default':     'false',
        'description': 'Launch moveit_servo?',
        'choices':     ['true', 'false', 'True', 'False']
    },
    {
        'name':        'sim',
        'default':     'false',
        'description': 'Use simulation time if true',
        'choices':     ['true', 'false', 'True', 'False']
    },
    {
        'name':        'vis',
        'default':     'false',
        'description': 'Launch rviz2 if true',
        'choices':     ['true', 'false', 'True', 'False']
    },
    {
        'name':        'warehouse_sqlite_path',
        'default':     os.path.expanduser('~/.ros/warehouse_ros.sqlite'),
        'description': 'Path where the warehouse database should be stored'
    },
    {
        'name':        'publish_robot_description_semantic',
        'default':     'true',
        'description': 'MoveGroup publishes robot description semantic'
    },
]

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def launch_setup(context):
    moveit_configs = MoveItConfigsBuilder(robot_name='aist_base_scene',
                                          package_name='aist_moveit_config') \
                    .robot_description_semantic(Path('config')
                                                / 'aist_base_scene.srdf',
                                                {'name': 'aist_base_scene'}) \
                    .to_moveit_configs()
    wait_robot_description = Node(package='ur_robot_driver',
                                  executable='wait_for_robot_description',
                                  output='screen')
    servo_yaml = load_yaml('aist_moveit_config', 'config/ur_servo.yaml')

    return [
        wait_robot_description,
        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_robot_description,
                on_exit=[
                    Node(package='moveit_ros_move_group',
                         executable='move_group',
                         output='screen',
                         parameters=[
                             moveit_configs.to_dict(),
                             {
                                 'warehouse_plugin':
                                 'warehouse_ros_sqlite::DatabaseConnection',
                                 'warehouse_host':
                                 LaunchConfiguration('warehouse_sqlite_path'),
                                 'use_sim_time': LaunchConfiguration('sim'),
                                 'publish_robot_description_semantic':
                                 LaunchConfiguration(
                                     'publish_robot_description_semantic'),
                             }
                         ]),
                    Node(condition=IfCondition(
                                       LaunchConfiguration('launch_servo')),
                         package='moveit_servo',
                         executable='servo_node',
                         parameters=[
                             moveit_configs.to_dict(),
                             {'moveit_servo': servo_yaml}
                         ],
                         output='screen'),
                    IncludeLaunchDescription(
                        PathJoinSubstitution(
                            [FindPackageShare('aist_moveit_config'), 'launch',
                             'moveit_rviz.launch.py']),
                        condition=IfCondition(LaunchConfiguration('vis'))),
                ]
            ))]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
