from launch                     import LaunchDescription
from launch.actions             import IncludeLaunchDescription, OpaqueFunction
from launch.conditions          import IfCondition, UnlessCondition
from launch.substitutions       import (LaunchConfiguration,
                                        PathJoinSubstitution)
from launch_ros.actions         import Node
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
    {
        'name':        'rviz_config_file',
        'default':     PathJoinSubstitution([FindPackageShare('aist_bringup'),
                                             'config',
                                             [LaunchConfiguration('config'),
                                              '.rviz']])
    }
]

def launch_setup(context):
    return [
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('aist_bringup'), 'launch',
                                  'ros2_controllers.launch.py']),
            launch_arguments=[('config', LaunchConfiguration('config')),
                              ('scene',  LaunchConfiguration('scene')),
                              ('sim',    LaunchConfiguration('sim'))]),
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('aist_bringup'), 'launch',
                                  'ros_gz_bridge.launch.py']),
            launch_arguments=[('config', LaunchConfiguration('config'))],
            condition=IfCondition(LaunchConfiguration('sim'))),
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('aist_bringup'), 'launch',
                                  'extra_drivers.launch.py']),
            launch_arguments=[('config', LaunchConfiguration('config'))],
            condition=UnlessCondition(LaunchConfiguration('sim'))),
        Node(condition=IfCondition(LaunchConfiguration('vis')),
             package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['-d', LaunchConfiguration('rviz_config_file')],
             output='screen')]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
