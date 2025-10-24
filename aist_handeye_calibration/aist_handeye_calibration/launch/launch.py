from launch                     import LaunchDescription
from launch.actions             import (SetLaunchConfiguration,
                                        IncludeLaunchDescription,
                                        OpaqueFunction)
from launch.conditions          import IfCondition
from launch.substitutions       import (LaunchConfiguration,
                                        PathJoinSubstitution)
from launch_ros.actions         import Node, LoadComposableNodes
from launch_ros.substitutions   import FindPackageShare
from launch_ros.descriptions    import ComposableNode
from aist_bringup.launch_common import (declare_launch_arguments,
                                        load_config, get_camera_props)

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
        'name':        'container',
        'default':     'cameeras_container',
        'description': 'name of the component container for cameras'
    },
]


def launch_setup(context):
    camera_name = LaunchConfiguration('camera_name').perform(context)
    camera_type = load_config(context).get('cameras', {})[camera_name]['type']

    return [
        SetLaunchConfiguration(
            'config_file',
            PathJoinSubstitution([
                FindPackageShare('aist_handeye_calibration'), 'config',
                [LaunchConfiguration('camera_name'), '.yaml']])),
        LoadComposableNodes(
            target_container=LaunchConfiguration('container'),
            composable_node_descriptions=[
                ComposableNode(
                    name='handeye_calibrator',
                    package='aist_handeye_calibration',
                    plugin='aist_handeye_calibration::Calibrator',
                    parameters=[LaunchConfiguration('config_file')],
                    remappings=[('pose', 'detector_3d/pose')],
                    extra_arguments=[{'use_intra_process_comms': True}])
            ]),
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('aist_aruco_ros'), 'launch',
                                  'detector_3d.launch.py']),
            launch_arguments=[
                ('detector_name', 'detector_3d'),
                ('camera_name',   camera_name),
                ('camera_type',   camera_type),
                ('config_file',   LaunchConfiguration('config_file')),
                ('container',     LaunchConfiguration('container')),
            ]),
    ]

    return actions

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
