from launch                     import LaunchDescription
from launch.actions             import IncludeLaunchDescription, OpaqueFunction
from launch.conditions          import IfCondition, UnlessCondition
from launch.substitutions       import (LaunchConfiguration,
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
        'name':        'scene',
        'default':     '',
        'description': 'Name of the scene'
    },
    {
        'name':        'sim',
        'default':     'false',
        'description': 'Launch gz if true',
        'choices':     ['true', 'false', 'True', 'False']
    },
    {
        'name':        'vis',
        'default':     LaunchConfiguration('sim'),
        'description': 'Launch rviz2 if true',
        'choices':     ['true', 'false', 'True', 'False']
    },
]

def launch_setup(context):
    return [
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('aist_bringup'), 'launch',
                                  'ros2_controllers.launch.py'])),
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('aist_bringup'), 'launch',
                                  'ros_gz_bridge.launch.py']),
            condition=IfCondition(LaunchConfiguration('sim'))),
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('aist_bringup'), 'launch',
                                  'extra_drivers.launch.py']),
            condition=UnlessCondition(LaunchConfiguration('sim'))),
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare([LaunchConfiguration('config'),
                                   '_moveit_config']),
                 'launch', 'move_group.launch.py'])),
    ]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
