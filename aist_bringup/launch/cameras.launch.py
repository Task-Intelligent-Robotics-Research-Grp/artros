from launch                     import LaunchDescription
from launch.actions             import IncludeLaunchDescription, OpaqueFunction
from launch.substitutions       import (LaunchConfiguration,
                                        PathJoinSubstitution)
from launch_ros.actions         import Node
from launch_ros.substitutions   import FindPackageShare
from aist_bringup.launch_common import (declare_launch_arguments,
                                        load_config, get_camera_props)

launch_arguments = [
    {
        'name':        'param_file',
        'default':     PathJoinSubstitution([
                           FindPackageShare('aist_brignup'), 'config',
                           'cameras.yaml']),
        'description': 'abolute path to YAML file for configuring cameras'
    },
    {
        'name':        'container',
        'default':     'cameras_container',
        'description': 'name of internal or external component container'
    },
    {
        'name':        'log_level',
        'default':     'info',
        'description': 'debug log level',
        'choices':     ['debug', 'info', 'warn', 'error', 'fatal']
    },
    {
        'name':        'output',
        'default':     'screen',
        'description': 'pipe node output',
        'choices':     ['screen', 'log', 'both']
    },
]


def launch_setup(context):
    config = load_config(context)
    camera_param_file = PathJoinSubstitution([
                            FindPackageShare('aist_bringup'), 'config',
                            'cameras.yaml'])
    actions = [
        Node(name=LaunchConfiguration('container'),
             package='rclcpp_components',
             executable='component_container_mt',
             output=LaunchConfiguration('output'),
             arguments=['--ros-args', '--log-level',
                        LaunchConfiguration('log_level')]),
    ]
    for camera_name, camera_config in config['cameras'].items():
        camera_props = get_camera_props(camera_config['type'])
        actions.append(
            IncludeLaunchDescription(
                camera_props['launch_file'],
                launch_arguments=[
                    ('camera_name',        camera_name),
                    ('external_container', 'true'),
                    ('container',          LaunchConfiguration('container'))
                ]))
    return actions

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
