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
    {'name':        'config_file',
     'default':     '',
     'description': 'path to YAML file for configuring the controller'},
    {'name':        'dynamixel_info',
     'default':     '',
     'description': 'path to YAML file for configuring fastening tools'},
    {'name':        'container',
     'default':     'screw_tools_container',
     'description': 'name of component container'},
    {'name':        'log_level',
     'default':     'info',
     'description': 'debug log level',
     'choices':     ['debug', 'info', 'warn', 'error', 'fatal']},
    {'name':        'output',
     'default':     'both',
     'description': 'pipe node output',
     'choices':     ['screen', 'log', 'both']}]

def get_node_names(config_file):
    with open(config_file, 'r') as f:
        conf = yaml.safe_load(f)
    return list(conf.keys())

def launch_setup(context):
    config_file = IfElseSubstitution(
                      EqualsSubstitution(
                          LaunchConfiguration('config_file'), ''),
                      PathJoinSubstitution(
                          [ThisLaunchFileDir(), '..', 'config',
                           'screw_tool_fastening_controllers.yaml']),
                      LaunchConfiguration('config_file'))
    dxlinfo_file = IfElseSubstitution(
                       EqualsSubstitution(
                           LaunchConfiguration('dynamixel_info'), ''),
                       PathJoinSubstitution(
                           [ThisLaunchFileDir(), '..', 'config',
                            'screw_tool_dynamixel_info.yaml']),
                       LaunchConfiguration('dynamixel_info'))

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
