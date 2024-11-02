import yaml
from launch                            import LaunchDescription
from launch.actions                    import (DeclareLaunchArgument,
                                               OpaqueFunction)
from launch.substitutions              import (LaunchConfiguration,
                                               PathJoinSubstitution,
                                               ThisLaunchFileDir)
from launch_ros.actions                import LoadComposableNodes
from launch_ros.descriptions           import ComposableNode
from launch.launch_description_sources import PythonLaunchDescriptionSource

launch_arguments = [
    {'name':        'name',
     'default':     'aruco_multi_detector',
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
     'description': 'name of external component container'}]

parameter_arguments = [
    {'name':        'camera_names',
     'default':     'realsense',
     'description': 'list of camera names'},
    {'name':        'marker_map_dir',
     'default':     PathJoinSubstitution([ThisLaunchFileDir(),
                                          '..', 'config']),
     'description': 'directory name containing marker map'},
    {'name':        'marker_map',
     'default':     'aruco-26-70x70-5',
     'description': 'name of marker map'},
    {'name':        'reference_frame',
     'default':     '',
     'description': 'marker frame ID'},
    {'name':        'marker_frame',
     'default':     'marker_frame',
     'description': 'marker frame ID'}]

def declare_launch_arguments(args, defaults={}):
    num_to_str = lambda x : str(x) if isinstance(x, (bool, int, float)) else x
    return [DeclareLaunchArgument(
                arg['name'],
                default_value=num_to_str(defaults.get(arg['name'],
                                                      arg['default'])),
                description=arg['description']) \
            for arg in args]

def set_configurable_parameters(args):
    return dict([(arg['name'], LaunchConfiguration(arg['name'])) \
                 for arg in args])

def load_parameters(config_file):
    if config_file == '':
        return {}
    with open(config_file, 'r') as f:
        return yaml.load(f, Loader=yaml.SafeLoader)

def launch_setup(context, param_args):
    camera_names = LaunchConfiguration('camera_names').preform(context).split()
    remappings   = [(camera_name + '/image',
                     camera_name + LaunchConfiguration(
                                       'image_topic').perform(context)) \
                    for camera_name in camera_names]
    params       = load_parameters(
                       LaunchConfiguration('config_file').perform(context))
    actions      = declare_launch_arguments(param_args, params)
    params      |= set_configurable_parameters(param_args)
    actions     += [LoadComposableNodes(
                        target_container=LaunchConfiguration('container'),
                        composable_node_descriptions=[
                            ComposableNode(
                                name=LaunchConfiguration('name'),
                                package='aist_aruco_ros',
                                plugin='aist_aruco_ros::MultiDetector',
                                parameters=[params],
                                remappings=remappings,
                                extra_arguments=[
                                    {'use_intra_process_comms': True}])])]
    return actions

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup,
                                             args=[parameter_arguments])])
