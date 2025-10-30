from launch                     import LaunchDescription
from launch.actions             import (SetLaunchConfiguration,
                                        IncludeLaunchDescription,
                                        OpaqueFunction)
from launch.substitutions       import (LaunchConfiguration,
                                        PathJoinSubstitution)
from launch.conditions          import UnlessCondition
from launch_ros.actions         import Node, LoadComposableNodes
from launch_ros.substitutions   import FindPackageShare
from launch_ros.descriptions    import ComposableNode
from aist_bringup.launch_common import declare_launch_arguments, load_config

launch_arguments = [
    {
        'name':        'config',
        'default':     'aist',
        'description': 'Name of the hardware configuration'
    },
    {
        'name':        'camera_name',
        'default':     'a_motioncam',
        'description': 'name of the camera to be calibrated'
    },
    {
        'name':        'external_container',
        'default':     'false',
        'description': 'use external container launched in advance',
        'choices':     ['true', 'false', 'True', 'False']
    },
    {
        'name':        'container',
        'default':     'handeye_calibrator_container',
        'description': 'name of the component container for cameras'
    },
    {
        'name':        'check',
        'default':     'false',
        'description': 'Check calibration result if true'
    },
    {
        'name':        'log_level',
        'default':     'info',
        'description': 'debug log level',
        'choices':     ['debug', 'info', 'warn', 'error', 'fatal']
    },
    {
        'name':        'output',
        'default':     'log',
        'description': 'pipe node output',
        'choices':     ['screen', 'log', 'both']
    },
    {
        'name':        'sim',
        'default':     'false',
        'description': 'Use simulation time if true',
        'choices':     ['true', 'false', 'True', 'False']
    },
]


def launch_setup(context):
    camera_name = LaunchConfiguration('camera_name').perform(context)
    camera_type = load_config(context).get('cameras', {})[camera_name]['type']

    client_node = Node(name='run_calibration',
                       package='aist_handeye_calibration',
                       executable='run_calibration.py',
                       parameters=[
                           LaunchConfiguration('params_file'),
                           {'config_file':
                            PathJoinSubstitution([
                                FindPackageShare('aist_bringup'), 'config',
                                [LaunchConfiguration('config'), '.yaml']]),
                            'use_sim_time': LaunchConfiguration('sim')}
                       ],
                       prefix=['xterm -fn 7x14 -sb -geometry 80x60 -e'],
                       output=LaunchConfiguration('output'),
                       arguments=['--ros-args', '--log-level',
                                  LaunchConfiguration('log_level')])

    return [
        SetLaunchConfiguration(
            'params_file',
            PathJoinSubstitution([
                FindPackageShare('aist_handeye_calibration'), 'config',
                [LaunchConfiguration('camera_name'), '.yaml']])),
        Node(name=LaunchConfiguration('container'),
             package='rclcpp_components',
             executable='component_container_mt',
             output=LaunchConfiguration('output'),
             arguments=['--ros-args', '--log-level',
                        LaunchConfiguration('log_level')],
             condition=UnlessCondition(
                           LaunchConfiguration('external_container'))),
        LoadComposableNodes(
            target_container=LaunchConfiguration('container'),
            composable_node_descriptions=[
                ComposableNode(
                    name='handeye_calibrator',
                    package='aist_handeye_calibration',
                    plugin='aist_handeye_calibration::Calibrator',
                    parameters=[LaunchConfiguration('params_file')],
                    remappings=[('pose', 'detector_3d/pose')],
                    extra_arguments=[{'use_intra_process_comms': True}])
            ]),
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('aist_aruco_ros'), 'launch',
                                  'detector_3d.launch.py']),
            launch_arguments=[
                ('detector_name',      'detector_3d'),
                ('camera_name',        camera_name),
                ('camera_type',        camera_type),
                ('config_file',        LaunchConfiguration('params_file')),
                # ('external_container', LaunchConfiguration('external_container')),
                # ('container',     LaunchConfiguration('container')),
                ('external_container', 'true'),
                ('container',          'cameras_container'),
            ]),
        client_node,
    ]

    return actions

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
