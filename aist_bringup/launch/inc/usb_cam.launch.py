from launch                     import LaunchDescription
from launch.actions             import OpaqueFunction
from launch.substitutions       import (LaunchConfiguration,
                                        PathJoinSubstitution)
from launch.conditions          import UnlessCondition
from launch_ros.actions         import Node, LoadComposableNodes
from launch_ros.substitutions   import FindPackageShare
from launch_ros.descriptions    import ComposableNode
from aist_bringup.launch_common import declare_launch_arguments

launch_arguments = [
    {
        'name':        'name',
        'default':     'live_camera',
        'description': 'Name of the camera'
    },
    {
        'name':        'external_container',
        'default':     'false',
        'description': 'use existing external container',
        'choices':     ['true', 'false', 'True', 'False']
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
                    namespace=LaunchConfiguration('name'),
                    name='usb_cam',
                    package='usb_cam',
                    plugin='usb_cam::UsbCamNode',
                    parameters=[
                        PathJoinSubstitution([
                            FindPackageShare('aist_bringup'),
                            'config', 'devices', 'devices.yaml']),
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}])])]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
