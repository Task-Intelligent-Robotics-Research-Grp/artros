from launch                     import LaunchDescription
from launch.actions             import OpaqueFunction
from launch.substitutions       import (LaunchConfiguration,
                                        PathJoinSubstitution)
from launch_ros.actions         import Node
from launch_ros.substitutions   import FindPackageShare
from aist_bringup.launch_common import declare_launch_arguments


launch_arguments = [
    {
        'name':        'param_file',
        'default':     PathJoinSubstitution([
                           FindPackageShare('aist_collision_object_manager'),
                           'config', 'default.yaml']),
        'description': 'Absolute path to YAML configuration file'
    },
    {
        'name':        'log_level',
        'default':     'info',
        'description': 'debug log level',
        'choices':     ['debug', 'info', 'warn', 'error', 'fatal']
    },
    {
        'name':        'output',
        'default':     'both',
        'description': 'pipe node output',
        'choices':     ['screen', 'log', 'both']
    }
]

def launch_setup(context):
    return [
        Node(name='collision_object_manager',
             package='aist_collision_object_manager',
             executable='collision_object_manager',
             parameters=[
                 LaunchConfiguration('param_file')
             ],
             arguments=[
                 '--ros-args', '--log-level',
                 LaunchConfiguration('log_level')
             ],
             output=LaunchConfiguration('output'))
    ]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
