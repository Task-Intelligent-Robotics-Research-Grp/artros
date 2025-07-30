import yaml
from launch                   import LaunchDescription
from launch.actions           import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions     import (LaunchConfiguration,
                                      PathJoinSubstitution, ThisLaunchFileDir)
from launch_ros.actions       import Node, LoadComposableNodes
from launch_ros.descriptions  import ComposableNode

launch_arguments = [
    {'name':        'namespace',
     'default':     '',
     'description': 'namespace of controllers'},
    {'name':        'config_file',
     'default':     '',
     'description': 'path to YAML file for configuring gripper'},
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

def get_node_names(conf_file_path):
    with open(conf_file_path, 'r') as f:
        conf = yaml.safe_load(f)
    return list(conf.keys())

def launch_setup(context):
    conf_file_path = PathJoinSubstitution(
                         [ThisLaunchFileDir(), '..', 'config',
                          'precision_gripper_controller.yaml'])
    node_names = get_node_names(conf_file_path.perform(context))

    return [Node(namespace=LaunchConfiguration('namespace'),
                 name=node_names[0],
                 package='dynamixel_workbench_controllers',
                 executable='dynamixel_workbench_controllers_node',
                 parameters=[conf_file_path],
                 output=LaunchConfiguration('output'),
                 arguments=['--ros-args', '--log-level',
                            LaunchConfiguration('log_level')]),
            Node(namespace=LaunchConfiguration('namespace'),
                 name=node_names[1],
                 package='aist_precision_gripper',
                 executable='precision_gripper_controller',
                 parameters=[conf_file_path],
                 output='screen',
                 arguments=['--ros-args', '--log-level',
                            LaunchConfiguration('log_level')])]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
