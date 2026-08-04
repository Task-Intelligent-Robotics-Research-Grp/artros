from launch                     import LaunchDescription
from launch.actions             import IncludeLaunchDescription, OpaqueFunction
from launch.conditions          import IfCondition, UnlessCondition
from launch.substitutions       import (LaunchConfiguration, NotSubstitution,
                                        PathJoinSubstitution)
from launch_ros.substitutions   import FindPackageShare
from aist_bringup.launch_common import declare_launch_arguments


launch_arguments = [
    {
        'name':        'config',
        'default':     'aist',
        'description': 'Name of the hardware configuration'
    },
    {
        'name':        'settings_file',
        'default':     PathJoinSubstitution([
                           FindPackageShare('aist_routines'), 'config',
                           'default.yaml']),
        'description': 'Name of the hardware configuration'
    },
    {
        'name':        'sim',
        'default':     'false',
        'description': 'Do not launch cameras if true',
        'choices':     ['true', 'false', 'True', 'False']
    },
    {
        'name':        'vis',
        'default':     NotSubstitution(LaunchConfiguration('sim')),
        'description': 'Launch rviz2 if true',
        'choices':     ['true', 'false', 'True', 'False']
    },
]

def launch_setup(context):
    return [
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare('aist_bringup'),
                 'launch', 'cameras.launch.py']),
            condition=UnlessCondition(LaunchConfiguration('sim'))),
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare([LaunchConfiguration('config'),
                                   '_moveit_config']),
                 'launch', 'moveit_rviz.launch.py']),
            condition=IfCondition(LaunchConfiguration('vis'))),
    ]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
