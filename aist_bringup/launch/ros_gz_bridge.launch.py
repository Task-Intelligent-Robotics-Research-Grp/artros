import shutil
from launch                     import LaunchDescription
from launch.actions             import SetLaunchConfiguration, OpaqueFunction
from launch.substitutions       import (LaunchConfiguration,
                                        PathJoinSubstitution)
from launch_ros.actions         import Node
from launch_ros.substitutions   import FindPackageShare
from aist_bringup.launch_common import (declare_launch_arguments,
                                        load_config, get_camera_props,
                                        instantiate_file)

launch_arguments = [
    {
        'name':        'config',
        'default':     'aist',
        'description': 'Name of the hardware configuration'
    },
    {
        'name':        'container',
        'default':     'cameras_container',
        'description': 'name of the component container for cameras'
    },
]


def launch_setup(context):
    bridge_config_file = '/tmp/camera_bridge.yaml'

    config = load_config(context)
    shutil.copy(PathJoinSubstitution(
                    [FindPackageShare('aist_bringup'), 'config',
                     'templates', 'clock_bridge.yaml']).perform(context),
                bridge_config_file)
    for camera_name, camera_config in config.get('cameras', {}).items():
        camera_props = get_camera_props(camera_config['type'])
        SetLaunchConfiguration('camera_name', camera_name).execute(context)
        if 'cloud_topic' in camera_props:
            SetLaunchConfiguration('cloud_topic', camera_props['cloud_topic'])\
                .execute(context)
        if 'depth_topic' in camera_props:
            SetLaunchConfiguration('depth_topic', camera_props['depth_topic'])\
                .execute(context)
        if 'cinfo_topic' in camera_props:
            SetLaunchConfiguration('cinfo_topic', camera_props['cinfo_topic'])\
                .execute(context)
        if 'color_topic' in camera_props:
            SetLaunchConfiguration('color_topic', camera_props['color_topic'])\
                .execute(context)
        instantiate_file(context, camera_props['gz_bridge_template'],
                         bridge_config_file, True)

    return [
        Node(name=LaunchConfiguration('container'),
             package='rclcpp_components',
             executable='component_container_mt',
             output='screen'),
        Node(package='ros_gz_bridge',
             executable='parameter_bridge',
             parameters=[{'config_file': bridge_config_file}],
             output='screen')
    ]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
