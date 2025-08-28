import yaml
from launch                   import LaunchDescription
from launch.actions           import (DeclareLaunchArgument, OpaqueFunction)
from launch.substitutions     import (LaunchConfiguration, ThisLaunchFileDir,
                                      PathJoinSubstitution, EqualsSubstitution,
                                      IfElseSubstitution)
from launch_ros.actions       import Node, LoadComposableNodes
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions  import ComposableNode

launch_arguments = [
    {'name':        'name',
     'default':     'multi_detector',
     'description': 'name of detector'},
    {'name':        'image_topic',
     'default':     '/color/image_raw',
     'description': 'topic name of intensity/color image'},
    {'name':        'camera_info_topic',
     'default':     '/color/camera_info',
     'description': 'topic name of camera_info'},
    {'name':        'config_file',
     'default':     '',
     'description': 'path to YAML file for configuring detector'},
    {'name':        'container',
     'default':     'my_container',
     'description': 'name of external component container'},
    {'name':        'log_level',
     'default':     'info',
     'description': 'debug log level',
     'choices':     ['debug', 'info', 'warn', 'error', 'fatal']},
    {'name':        'output',
     'default':     'both',
     'description': 'pipe node output',
     'choices':     ['screen', 'log', 'both']}]

parameter_arguments = [
    {'name':        'marker_map_dir',
     'default':     PathJoinSubstitution([ThisLaunchFileDir(),
                                          '..', 'config']),
     'description': 'directory name containing marker map'},
    {'name':        'marker_map',
     'default':     'aruco-26-70x70-5',
     'description': 'name of marker map'}]

def declare_launch_arguments(args):
    return [DeclareLaunchArgument(arg['name'],
                                  default_value=arg.get('default'),
                                  description=arg.get('description'),
                                  choices=arg.get('choices')) \
            for arg in args]

def set_configurable_parameters(args):
    return {arg['name']: LaunchConfiguration(arg['name']) for arg in args}

def get_node_names(config_file):
    with open(config_file, 'r') as f:
        conf = yaml.safe_load(f)
    return list(conf.keys())

def launch_setup(context, param_args):
    config_file   = IfElseSubstitution(
                        EqualsSubstitution(
                            LaunchConfiguration('config_file'), ''),
                        PathJoinSubstitution(
                            [FindPackageShare('aist_aruco_ros'), 'config',
                             'test_multi_detector.yaml']),
                        LaunchConfiguration('config_file'))
    config_params = set_configurable_parameters(param_args)
    camera_names  = get_node_names(config_file.perform(context))[1:]
    composable_nodes = []
    remappings = []
    for camera_name in camera_names:
        composable_nodes.append(
            ComposableNode(
                namespace='',
                name=camera_name,
                package='realsense2_camera',
                plugin='realsense2_camera::RealSenseNodeFactory',
                parameters=[config_file],
                extra_arguments=[{'use_intra_process_comms': True}]))
        remappings.append((camera_name + '/image',
                           camera_name + '/color/image_raw'))
    print(remappings)
    composable_nodes.append(
        ComposableNode(
            name='multi_detector',
            package='aist_aruco_ros',
            plugin='aist_aruco_ros::MultiDetector',
            parameters=[config_file, config_params],
            remappings=remappings,
            extra_arguments=[{'use_intra_process_comms': True}]))

    return [Node(name=LaunchConfiguration('container'),
                 package='rclcpp_components',
                 executable='component_container_mt',
                 output=LaunchConfiguration('output'),
                 arguments=['--ros-args', '--log-level',
                            LaunchConfiguration('log_level')]),
            LoadComposableNodes(
                target_container=LaunchConfiguration('container'),
                composable_node_descriptions=composable_nodes),
            Node(name='rviz', package='rviz2', executable='rviz2',
                 output='screen',
                 arguments=['-d',
                            PathJoinSubstitution([
                                ThisLaunchFileDir(),
                                'test_multi_detector.rviz'])]),
            Node(name='rqt_reconfigure', package='rqt_reconfigure',
                 executable='rqt_reconfigure', output='screen')]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments +
                                                      parameter_arguments) + \
                             [OpaqueFunction(function=launch_setup,
                                             args=[parameter_arguments])])
