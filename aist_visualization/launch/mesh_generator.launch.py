from launch                     import LaunchDescription
from launch.actions             import OpaqueFunction
from launch.substitutions       import (LaunchConfiguration,
                                        PathJoinSubstitution)
from launch.conditions          import UnlessCondition
from launch_ros.actions         import Node, LoadComposableNodes
from launch_ros.substitutions   import FindPackageShare
from launch_ros.descriptions    import ComposableNode
from aist_bringup.launch_common import (declare_launch_arguments,
                                        get_camera_props)

launch_arguments = [
    {
        'name':        'camera_name',
        'default':     'live_camera',
        'description': 'name of the camera'
    },
    {
        'name':        'camera_type',
        'default':     'USBCamera',
        'description': 'type of the camera'
    },
    {
        'name':        'param_file',
        'default':     PathJoinSubstitution([
                           FindPackageShare('aist_visualization'), 'config',
                           'default.yaml']),
        'description': 'abolute path to YAML file for configuring camera'
    },
    {
        'name':        'external_container',
        'default':     'false',
        'description': 'use external container launched in advance',
        'choices':     ['true', 'false', 'True', 'False']
    },
    {
        'name':        'container',
        'default':     'cameras_container',
        'description': 'name of the component container'
    },
    {
        'name':        'log_level',
        'default':     'info',
        'description': 'debug log level',
        'choices':     ['debug', 'info', 'warn', 'error', 'fatal']
    },
    {
        'name':        'output',
        'default':     'log',
        'description': 'pipe node output',
        'choices':     ['screen', 'log', 'both']
    },
]


def launch_setup(context):
    camera_props = get_camera_props(LaunchConfiguration('camera_type') \
                                    .perform(context))
    return [
        Node(name=LaunchConfiguration('container'),
             package='rclcpp_components',
             executable='component_container_mt',
             output=LaunchConfiguration('output'),
             arguments=['--ros-args', '--log-level',
                        LaunchConfiguration('log_level')],
             condition=UnlessCondition(
                           LaunchConfiguration('external_container'))),
        LoadComposableNodes(
            target_container=LaunchConfiguration('container'),
            composable_node_descriptions=[
                ComposableNode(
                    name='mesh_generator',
                    package='aist_visualization',
                    plugin='aist_visualization::MeshGenerator',
                    parameters=[LaunchConfiguration('param_file')],
                    remappings=[
                        ('camera_info',
                         [LaunchConfiguration('camera_name'), '/',
                          camera_props['cinfo_topic']])],
                    extra_arguments=[{'use_intra_process_comms': True}])
            ]),
    ]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
