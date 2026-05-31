from launch                     import LaunchDescription
from launch.actions             import IncludeLaunchDescription, OpaqueFunction
from launch.substitutions       import (LaunchConfiguration,
                                        PathJoinSubstitution)
from launch_ros.substitutions   import FindPackageShare
from aist_bringup.launch_common import declare_launch_arguments


launch_arguments = [
    {
        'name':        'settings_file',
        'default':     PathJoinSubstitution([
                           FindPackageShare('aist_routines'), 'config',
                           'default.yaml']),
        'description': 'Name of the hardware configuration'
    },
]

def launch_setup(context):
    return [
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare('aist_bringup'), 'launch',
                 'cameras.launch.py'])),
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare('aist_collision_object_manager'),
                 'launch', 'launch.py']),
            launch_arguments=[
                ('param_file',  LaunchConfiguration('settings_file')),
            ]),
    ]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
