from launch                     import LaunchDescription
from launch.actions             import IncludeLaunchDescription, OpaqueFunction
from launch.substitutions       import (LaunchConfiguration,
                                        PathJoinSubstitution)
from launch_ros.actions         import Node
from launch_ros.substitutions   import FindPackageShare
from aist_bringup.launch_common import (declare_launch_arguments,
                                        get_camera_props)

CAMERA_TYPES = {
    'realsense': 'RealsenseCamera',
    'phoxi':     'PhoXiCamera'
}

launch_arguments = [
    {
        'name':        'camera_name',
        'default':     'realsense',
        'choices':     ['realsense', 'phoxi'],
        'description': 'name of the camera'
    },
    {
        'name':        'id',
        'default':     '""',
        'description': 'unique ID of camera'
    },
    {
        'name':        'param_file',
        'default':     PathJoinSubstitution(
                           [FindPackageShare('aist_depth_filter'), 'config',
                            'default.yaml']),
        'description': 'absolute path to YAML file for configuring filter'
    },
    {
        'name':        'subscribe_normal',
        'default':     'false',
        'description': 'subscribe normal image from the camera if true',
        'choices':     ['true', 'false', 'True', 'False']
    },
    {
        'name':        'container',
        'default':     'cameras_container',
        'description': 'name of the component container'
    },
    {
        'name':        'log_level',
        'default':     'info',
        'description': 'debug log level',
        'choices':     ['debug', 'info', 'warn', 'error', 'fatal']
    },
    {
        'name':        'output',
        'default':     'both',
        'description': 'pipe node output',
        'choices':     ['screen', 'log', 'both']
    }
]


def launch_setup(context):
    camera_type  = CAMERA_TYPES[LaunchConfiguration('camera_name')\
                                .perform(context)]
    camera_props = get_camera_props(camera_type)

    return [
        IncludeLaunchDescription(
            camera_props['launch_file'],
            launch_arguments=[
                (camera_props['key_of_id'], LaunchConfiguration('id')),
                ('external_container',      'false'),
            ]),
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('aist_depth_filter'),
                                  'launch', 'launch.py']),
            launch_arguments=[
                ('camera_type',        camera_type),
                ('external_container', 'true'),
            ]),
        Node(name='rviz',
             package='rviz2', executable='rviz2', output='screen',
             arguments=[
                 '-d',
                 PathJoinSubstitution(
                     [FindPackageShare('aist_depth_filter'), 'launch',
                      'depth_filter.rviz']),
             ]),
        Node(name='rqt_reconfigure', package='rqt_reconfigure',
             executable='rqt_reconfigure', output='screen')
    ]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
