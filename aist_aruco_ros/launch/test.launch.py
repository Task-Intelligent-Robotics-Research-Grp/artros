from launch                     import LaunchDescription
from launch.actions             import IncludeLaunchDescription, OpaqueFunction
from launch.substitutions       import (LaunchConfiguration, ThisLaunchFileDir,
                                        PathJoinSubstitution)
from launch_ros.actions         import Node, LoadComposableNodes
from launch_ros.substitutions   import FindPackageShare
from launch_ros.descriptions    import ComposableNode
from aist_bringup.launch_common import (declare_launch_arguments,
                                        get_device_props)

CAMERA_TYPES = {
    'realsense': 'RealsenseCamera',
    'phoxi':     'PhoXiCamera'
}

launch_arguments = [
    {
        'name':        'detector_type',
        'default':     'detector_3d',
        'choices':     ['detector_3d', 'multi_detector'],
        'description': 'type of the detector to be tested'
    },
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
        'default':     PathJoinSubstitution([ThisLaunchFileDir(),
                                             '..', 'config', 'default.yaml']),
        'description': 'absolute path to YAML file for configuring detector'
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
    actions      = []
    camera_types = []
    for camera_name in LaunchConfiguration('camera_name').perform(context)\
                                                         .split(','):
        camera_type  = CAMERA_TYPES[camera_name]
        camera_props = get_device_props(camera_type)
        camera_types.append(camera_type)
        actions.append(
            IncludeLaunchDescription(
                camera_props['launch_file'],
                launch_arguments=[
                    (camera_props['key_of_id'], LaunchConfiguration('id')),
                    ('external_container',      'true'),
                ]))

    actions += [
        Node(name=LaunchConfiguration('container'),
             package='rclcpp_components',
             executable='component_container_mt',
             output=LaunchConfiguration('output'),
             arguments=['--ros-args', '--log-level',
                        LaunchConfiguration('log_level')]),
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('aist_aruco_ros'), 'launch',
                                  [LaunchConfiguration('detector_type'),
                                   '.launch.py']]),
            launch_arguments=[
                ('detector_name', LaunchConfiguration('detector_type')),
                ('camera_type',   camera_types),
            ]),
        Node(name='rviz',
             package='rviz2', executable='rviz2', output='screen',
             arguments=[
                 '-d',
                 PathJoinSubstitution(
                     [FindPackageShare('aist_aruco_ros'), 'launch',
                      [LaunchConfiguration('detector_type'), '.rviz']]),
             ]),
        Node(name='rqt_reconfigure', package='rqt_reconfigure',
             executable='rqt_reconfigure', output='screen')
    ]

    return actions

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
