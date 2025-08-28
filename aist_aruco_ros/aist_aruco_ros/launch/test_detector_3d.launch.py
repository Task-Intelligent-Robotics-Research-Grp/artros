from launch                   import LaunchDescription
from launch.actions           import (DeclareLaunchArgument,
                                      IncludeLaunchDescription,
                                      OpaqueFunction)
from launch.substitutions     import (LaunchConfiguration,
                                      PathJoinSubstitution,
                                      IfElseSubstitution,
                                      EqualsSubstitution)
from launch_ros.actions       import Node, LoadComposableNodes
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions  import ComposableNode


CAMERAS = {
    'realsense': {'package':           'realsense2_camera',
                  'launch_file':       'launch.py',
                  'key_of_id':         'serial_no',
                  'cloud_topic':       '/depth/color/points',
                  'depth_topic':       '/aligned_depth_to_color/image_raw',
                  'camera_info_topic': '/aligned_depth_to_color/camera_info',
                  'image_topic':       '/color/image_raw'},
    'phoxi':     {'package':           'aist_phoxi_camera',
                  'launch_file':       'launch.py',
                  'key_of_id':         'id',
                  'cloud_topic':       '/pointcloud',
                  'depth_topic':       '/depth_map',
                  'camera_info_topic': '/camera_info',
                  'image_topic':       '/texture'}}

launch_arguments = [
    {'name':        'camera_name',
     'default':     'realsense',
     'description': 'camera unique name'},
    {'name':        'id',
     'default':     '',
     'description': 'unique ID of camera'},
    {'name':        'config_file',
     'default':     '',
     'description': 'path to YAML file for configuring detector and cameras'},
    {'name':        'container',
     'default':     'my_container',
     'description': 'name of internal or external component container'},
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

def launch_setup(context, param_args):
    config_file   = IfElseSubstitution(
                        EqualsSubstitution(
                            LaunchConfiguration('config_file'), ''),
                        PathJoinSubstitution(
                            [FindPackageShare('aist_aruco_ros'), 'config',
                             'test_detector_3d.yaml']),
                        LaunchConfiguration('config_file'))
    config_params = set_configurable_parameters(param_args)
    camera_name   = LaunchConfiguration('camera_name').perform(context)
    camera        = CAMERAS[camera_name]

    return [IncludeLaunchDescription(
                PathJoinSubstitution([
                    FindPackageShare(camera['package']), 'launch',
                    camera['launch_file']]),
                launch_arguments=[
                    ('camera_name',       LaunchConfiguration('camera_name')),
                    ('config_file',       LaunchConfiguration('config_file')),
                    (camera['key_of_id'], LaunchConfiguration('id')),
                    ('container',         LaunchConfiguration('container')),
                    ('output',            LaunchConfiguration('output')),
                    ('log_level',         LaunchConfiguration('log_level'))]),
            LoadComposableNodes(
                target_container=LaunchConfiguration('container'),
                composable_node_descriptions=[
                    ComposableNode(
                        name='detector_3d',
                        package='aist_aruco_ros',
                        plugin='aist_aruco_ros::Detector3D',
                        parameters=[config_file, config_params],
                        remappings=[
                            ('/camera_info',
                             camera_name + camera['camera_info_topic']),
                            ('/depth', camera_name + camera['depth_topic']),
                            ('/image', camera_name + camera['image_topic'])],
                        extra_arguments=[{'use_intra_process_comms': True}])]),
            Node(name='rviz',
                 package='rviz2', executable='rviz2', output='screen',
                 arguments=['-d',
                            PathJoinSubstitution(
                                [FindPackageShare('aist_aruco_ros'), 'launch',
                                 camera_name + '.rviz'])]),
            Node(name='rqt_reconfigure', package='rqt_reconfigure',
                 executable='rqt_reconfigure', output='screen')]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments +
                                                      parameter_arguments) + \
                             [OpaqueFunction(function=launch_setup,
                                             args=[parameter_arguments])])
