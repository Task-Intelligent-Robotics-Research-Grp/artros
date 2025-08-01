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
     'description': 'path to YAML file for configuring gripper'},
    {'name':        'dynamixel_info',
     'default':     '',
     'description': 'path to YAML file for configuring dynamixel of gripper'},
    {'name':        'container',
     'default':     'precision_gripper_container',
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
                           'precision_gripper_controller.yaml']),
                      LaunchConfiguration('config_file'))
    dxlinfo_file = IfElseSubstitution(
                       EqualsSubstitution(
                           LaunchConfiguration('dynamixel_info'), ''),
                       PathJoinSubstitution(
                           [ThisLaunchFileDir(), '..', 'config',
                            'precision_gripper_dynamixel_info.yaml']),
                       LaunchConfiguration('dynamixel_info'))

    node_names = get_node_names(config_file.perform(context))
    composable_nodes = [
        ComposableNode(
            namespace=LaunchConfiguration('namespace'),
            name=node_names[0],
            package='dynamixel_workbench_controllers',
            plugin='dynamixel_workbench_controllers::DynamixelController',
            parameters=[config_file, {'dynamixel_info': dxlinfo_file}],
            extra_arguments=[{'use_intra_process_comms': True}]),
        ComposableNode(
            namespace=LaunchConfiguration('namespace'),
            name=node_names[1],
            package='aist_precision_gripper',
            plugin='aist_precision_gripper::PrecisionGripperController',
            parameters=[config_file],
            extra_arguments=[{'use_intra_process_comms': True}])]

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
