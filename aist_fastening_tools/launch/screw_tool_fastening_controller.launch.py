import yaml
from launch                     import LaunchDescription
from launch.actions             import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions       import (LaunchConfiguration, ThisLaunchFileDir,
                                        PathJoinSubstitution,
                                        EqualsSubstitution,
                                        IfElseSubstitution)
from launch_ros.actions         import Node, LoadComposableNodes
from launch_ros.descriptions    import ComposableNode
from aist_bringup.launch_common import declare_launch_arguments

launch_arguments = [
    {
        'name':        'name',
        'default':     'screw_tool_m4',
        'description': 'name of the screw tool'
    },
    {
        'name':        'usb_port',
        'default':     '/dev/ttyUSB0',
        'description': 'device name of the USB port'
    },
    {
        'name':        'motor_id',
        'default':     '3',
        'description': 'ID of the Dynamixel motor'
    },
    {
        'name':        'container_name',
        'default':     'screw_tool_fastening_container',
        'description': 'name of component container'
    },
    {
        'name':        'driver_name',
        'default':     'screw_tool_fastening_driver',
        'description': 'name of Dynamixel driver'
    },
    {
        'name':        'launch_container',
        'default':     'true',
        'description': 'launch container if true'
        'choices':     ['true', 'false', 'True', 'False']},
    },
    {
        'name':        'launch_driver',
        'default':     'true',
        'description': 'launch Dynamixel driver if true'
        'choices':     ['true', 'false', 'True', 'False']},
    },
    {
        'name':        'log_level',
        'default':     'info',
        'description': 'debug log level',
        'choices':     ['debug', 'info', 'warn', 'error', 'fatal']},
    {
        'name':        'output',
        'default':     'both',
        'description': 'pipe node output',
        'choices':     ['screen', 'log', 'both']
    }
]

def launch_setup(context):
    param_file = ParameterFile(PathJoinSubstitution(
                                 [FindPackageShare('aist_fastening_tools'),
                                  'config', 'screw_tool_controller.yaml']),
                               allow_substs=True)
    instantiate_file(context,
                     PathJoinSubstitution(
                                 [FindPackageShare('aist_fastening_tools'),
                                  'config', 'screw_tool_dynamixel_info.yaml']),
                     '/tmp/screw_tool_dynamixel_info.yaml')


    return [
        Node(name=LaunchConfiguration('container'),
             package='rclcpp_components',
             executable='component_container_mt',
             output=LaunchConfiguration('output'),
             arguments=['--ros-args', '--log-level',
                        LaunchConfiguration('log_level')]),
        LoadComposableNodes(
            target_container=LaunchConfiguration('container'),
            composable_node_descriptions=[
                ComposableNode(
                    name='screw_tools_fastening_driver',
                    package='dynamixel_workbench_controllers',
                    plugin='dynamixel_workbench_controllers::DynamixelController',
                    parameters=[param_file],
                    extra_arguments=[{'use_intra_process_comms': True}]),
                ComposableNode(
                    name=[LaunchConfiguration('name'), '_controller'],
                    package='aist_fastening_tools',
                    plugin='aist_fastening_tools::ScrewToolController',
                    parameters=[param_file],
                    extra_arguments=[{'use_intra_process_comms': True}])]
        )
    ]
    node_names = get_node_names(config_file.perform(context))
    composable_nodes = [
        ComposableNode(
            name=node_names[0],
            package='dynamixel_workbench_controllers',
            plugin='dynamixel_workbench_controllers::DynamixelController',
            parameters=[config_file, {'dynamixel_info': dxlinfo_file}],
            extra_arguments=[{'use_intra_process_comms': True}])]
    for node_name in node_names[1:]:
        composable_nodes.append(
            ComposableNode(
                name=node_name,
                package='aist_fastening_tools',
                plugin='aist_fastening_tools::ScrewToolController',
                parameters=[config_file],
                extra_arguments=[{'use_intra_process_comms': True}]))

    return [Node(name=LaunchConfiguration('container'),
                 package='rclcpp_components',
                 executable='component_container_mt',
                 output=LaunchConfiguration('output'),
                 arguments=['--ros-args', '--log-level',
                            LaunchConfiguration('log_level')]),
            LoadComposableNodes(
                target_container=LaunchConfiguration('container'),
                composable_node_descriptions=composable_nodes)]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
