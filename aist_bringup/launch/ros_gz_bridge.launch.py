import os, yaml
from launch                     import LaunchDescription
from launch.actions             import (SetLaunchConfiguration,
                                        OpaqueFunction)
from launch.substitutions       import (LaunchConfiguration,
                                        ThisLaunchFileDir,
                                        PathJoinSubstitution)
from launch_ros.actions         import Node
from aist_bringup.launch_common import (declare_launch_arguments,
                                        load_config,
                                        instantiate_config_file)

launch_arguments = [
    {
        'name':        'config',
        'default':     'aist',
        'description': 'Name of the hardware configuration'
    },
]


def launch_setup(context):
    bridge_config_file = '/tmp/camera_bridge.yaml'

    config = load_config(context)
    shutil.copy(PathJoinSubstitution(
                    [ThisLaunchFileDir(), '..', 'config',
                     'templates', 'clock_bridge.yaml']).perform(context),
                bridge_config_file)
    for camera_name, camera_config in config['cameras'].items():
        camera_props = get_camera_props(camera_config['type'])
        SetLaunchConfiguration('camera_name', camera_name).execute(context)
        SetLaunchConfiguration('cloud_topic',
                               camera_props['cloud_topic']).execute(context)
        SetLaunchConfiguration('depth_topic',
                               camera_props['depth_topic']).execute(context)
        SetLaunchConfiguration('cinfo_topic',
                               camera_props['cinfo_topic']).execute(context)
        SetLaunchConfiguration('color_topic',
                               camera_props['color_topic']).execute(context)
        instantiate_config_file(context,
                                PathJoinSubstitution(
                                    [ThisLaunchFileDir(), '..', 'config',
                                     'templates', 'camera_bridge.yaml']),
                                bridge_config_file, True)

    return [
        Node(package='ros_gz_bridge',
             executable='parameter_bridge',
             parameters=[{'config_file': bridge_config_file}],
             output='screen')]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
