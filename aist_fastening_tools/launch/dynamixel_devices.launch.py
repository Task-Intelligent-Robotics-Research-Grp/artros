from launch                     import LaunchDescription
from launch.actions             import OpaqueFunction
from launch.substitutions       import (LaunchConfiguration,
                                        PathJoinSubstitution)
from launch_ros.actions         import Node, LoadComposableNodes
from launch_ros.descriptions    import ComposableNode
from launch_ros.substitutions   import FindPackageShare
from aist_bringup.launch_common import declare_launch_arguments

launch_arguments = [
    {
        'name':        'param_file',
        'default':     PathJoinSubstitution(
                           [FindPackageShare('aist_fastening_tools'),
                            'config', 'default.yaml']),
        'description': 'absolute path to configuration file'
    },
    {
        'name':        'device_names',
        'default':     'screw_tool_m3,screw_tool_m4',
        'description': 'list of device names'
    },
    {
        'name':        'device_types',
        'default':     'ScrewTool,ScrewTool',
        'description': 'list of device names'
    },
    {
        'name':        'driver_ns',
        'default':     'screw_tools_driver',
        'description': 'name of the Dynamixel driver'
    },
    {
        'name':        'container',
        'default':     'screw_tools_container',
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
        'default':     'both',
        'description': 'pipe node output',
        'choices':     ['screen', 'log', 'both']
    }
]

PLUGINS = {
    'PrecisionTool': 'aist_fastening_tools::PrecisionToolController',
    'ScrewTool':     'aist_fastening_tools::ScrewToolController',
}

def launch_setup(context):
    composable_nodes = [
        ComposableNode(
            name=LaunchConfiguration('driver_ns'),
            package='dynamixel_workbench_controllers',
            plugin='dynamixel_workbench_controllers::DynamixelController',
            parameters=[LaunchConfiguration('param_file')],
            extra_arguments=[{'use_intra_process_comms': True}])
    ]

    for device_name, device_type \
          in zip(LaunchConfiguration('device_names').perform(context)
                 .split(','),
                 LaunchConfiguration('device_types').perform(context)
                 .split(',')):
        if device_type == 'ScrewTool':
            composable_nodes.append(
                ComposableNode(
                    name=device_name + '_controller',
                    package='aist_fastening_tools',
                    plugin='aist_fastening_tools::SuctionToolController',
                    parameters=[LaunchConfiguration('param_file')],
                    extra_arguments=[{'use_intra_process_comms': True}]))
            controller_name = device_name + '_fastening_controller'
        else:
            controller_name = device_name + '_controller'
        composable_nodes.append(
            ComposableNode(
                name=controller_name,
                package='aist_fastening_tools',
                plugin=PLUGINS[device_type],
                parameters=[LaunchConfiguration('param_file')],
                extra_arguments=[{'use_intra_process_comms': True}]))

    return [
        Node(name=LaunchConfiguration('container'),
             package='rclcpp_components',
             executable='component_container_mt',
             output=LaunchConfiguration('output'),
             arguments=['--ros-args', '--log-level',
                        LaunchConfiguration('log_level')]),
        LoadComposableNodes(
            target_container=LaunchConfiguration('container'),
            composable_node_descriptions=composable_nodes)
    ]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
