import yaml
from launch                  import LaunchDescription
from launch.actions          import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions    import (LaunchConfiguration, ThisLaunchFileDir,
                                     PathJoinSubstitution, EqualsSubstitution,
                                     IfElseSubstitution)
from launch_ros.actions      import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

launch_arguments = [
    {'name':        'namespace',
     'default':     '',
     'description': 'namespace of controllers'},
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
     'description': 'debug log level [DEBUG|INFO|WARN|ERROR|FATAL]'},
    {'name':        'output',
     'default':     'screen',
     'description': 'pipe node output [screen|log|both]'}]

def declare_launch_arguments(args):
    return [DeclareLaunchArgument(arg['name'],
                                  default_value=arg['default'],
                                  description=arg['description']) \
            for arg in args]

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
            namespace=LaunchConfiguration('namespace'),
            name=node_names[0],
            package='dynamixel_workbench_controllers',
            plugin='dynamixel_workbench_controllers::DynamixelController',
            parameters=[config_file, {'dynamixel_info': dxlinfo_file}],
            extra_arguments=[{'use_intra_process_comms': True}])]
    for node_name in node_names[1:]:
        composable_nodes.append(
            ComposableNode(
                namespace=LaunchConfiguration('namespace'),
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
