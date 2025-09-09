import os, yaml
from launch                            import LaunchDescription
from launch.actions                    import (SetLaunchConfiguration,
                                               DeclareLaunchArgument,
                                               IncludeLaunchDescription,
                                               OpaqueFunction,
                                               GroupAction)
from launch.conditions                 import IfCondition, UnlessCondition
from launch.substitutions              import (Command, FindExecutable,
                                               LaunchConfiguration,
                                               ThisLaunchFileDir,
                                               PathJoinSubstitution,
                                               IfElseSubstitution)
from launch_ros.actions                import Node
from launch_ros.substitutions          import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile
from aist_bringup.launch_common        import declare_launch_arguments

launch_arguments = [
    {'name':        'config',
     'default':     'aist',
     'description': 'Name of the hardware configuration'},
    {'name':        'scene',
     'default':     '',
     'description': 'Name of the scene'},
    {'name':        'sim',
     'default':     'false',
     'description': 'Launch gz if true',
     'choices':     ['true', 'false', 'True', 'False']},
    {'name':        'vis',
     'default':     LaunchConfiguration('sim'),
     'description': 'Launch rviz2 if true',
     'choices':     ['true', 'false', 'True', 'False']},
    {'name':        'rviz_config_file',
     'default':     PathJoinSubstitution([ThisLaunchFileDir(),
                                          [LaunchConfiguration('config'),
                                           '.rviz']])}
]

def launch_setup(context):
    return [
        IncludeLaunchDescription(
            PathJoinSubstitution([ThisLaunchFileDir(),
                                  'ros2_controllers.launch.py']),
            launch_arguments=[('config', LaunchConfiguration('config')),
                              ('scene',  LaunchConfiguration('scene')),
                              ('sim',    LaunchConfiguration('sim'))]),
        GroupAction(
            condition=UnlessCondition(LaunchConfiguration('sim')),
            actions=[
                IncludeLaunchDescription(
                    PathJoinSubstitution([ThisLaunchFileDir(),
                                          'extra_drivers.launch.py']),
                    launch_arguments=[
                        ('config', LaunchConfiguration('config'))])]),
        Node(condition=IfCondition(LaunchConfiguration('vis')),
             package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['-d', LaunchConfiguration('rviz_config_file')],
             output='screen')]


def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
