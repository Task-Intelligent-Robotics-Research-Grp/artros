import yaml
from launch                  import LaunchDescription
from launch.actions          import (DeclareLaunchArgument, OpaqueFunction,
                                     GroupAction)
from launch.substitutions    import (LaunchConfiguration, ThisLaunchFileDir,
                                     PathJoinSubstitution, EqualsSubstitution,
                                     IfElseSubstitution)
from launch.conditions       import IfCondition, UnlessCondition
from launch_ros.actions      import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

launch_arguments = [
    {'name':        'namespace',
     'default':     '',
     'description': 'namespace of the camera node'},
    {'name':        'multiplexer_name',
     'default':     '',
     'description': 'name of the multiplexer'},
    {'name':        'config_file',
     'default':     '',
     'description': 'path to YAML file for configuring the cameras'},
    {'name':        'active_camera_name',
     'default':     '',
     'description': 'path to YAML file for configuring fastening tools'},
    {'name':        'external_container',
     'default':     'false',
     'description': 'use existing external container'},
    {'name':        'container',
     'default':     'camera_multiplexer_container',
     'description': 'name of internal or external component container'},
    {'name':        'sim',
     'default':     'false',
     'description': 'use setting of gazebo simulation if true'},
    {'name':        'vis',
     'default':     'false',
     'description': 'launch Rviz if true'},
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
                           'realsense.yaml']),
                      LaunchConfiguration('config_file'))

    camera_names = get_node_names(config_file.perform(context))
    composable_nodes = []
    remappings = []
    for camera_name in camera_names:
        composable_nodes.append(
            ComposableNode(
                namespace=LaunchConfiguration('namespace'),
                name=camera_name,
                package='realsense2_camera',
                plugin='realsense2_camera::RealSenseNodeFactory',
                parameters=[config_file],
                extra_arguments=[{'use_intra_process_comms': True}],
                condition=UnlessCondition(LaunchConfiguration('sim'))))
        remappings += [(camera_name + '/depth',
                        camera_name + '/aligned_depth_to_color/image_raw'),
                       (camera_name + '/image',
                        camera_name + '/color/image_raw'),
                       (camera_name + '/camera_info',
                        camera_name + '/color/camera_info'),
                       (camera_name + '/pointcloud',
                        camera_name + '/depth/points')]
    composable_nodes.append(
        ComposableNode(
            namespace=LaunchConfiguration('namespace'),
            name=LaunchConfiguration('multiplexer_name'),
            package='aist_camera_multiplexer',
            plugin='aist_camera_multiplexer::Multiplexer',
            parameters=[{'camera_names': camera_names}],
            remappings=remappings,
            extra_arguments=[{'use_intra_process_comms': True}]))

    return [Node(name=LaunchConfiguration('container'),
                 package='rclcpp_components',
                 executable='component_container_mt',
                 output=LaunchConfiguration('output'),
                 arguments=['--ros-args', '--log-level',
                            LaunchConfiguration('log_level')],
                 condition=UnlessCondition(
                     LaunchConfiguration('external_container'))),
            LoadComposableNodes(
                target_container=LaunchConfiguration('container'),
                composable_node_descriptions=composable_nodes),
            GroupAction(
                condition=IfCondition(LaunchConfiguration('vis')),
                actions=[
                    Node(name='rviz', package='rviz2', executable='rviz2',
                         output='screen',
                         arguments=['-d',
                                    PathJoinSubstitution([
                                        ThisLaunchFileDir(),
                                        'realsense.rviz'])]),
                    Node(name='rqt_reconfigure', package='rqt_reconfigure',
                         executable='rqt_reconfigure', output='screen')])]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
