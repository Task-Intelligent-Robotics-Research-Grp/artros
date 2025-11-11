from launch                     import LaunchDescription
from launch.actions             import OpaqueFunction, SetLaunchConfiguration
from launch.substitutions       import (LaunchConfiguration,
                                        PathJoinSubstitution)
from launch.conditions          import UnlessCondition
from launch_ros.actions         import Node, LoadComposableNodes
from launch_ros.substitutions   import FindPackageShare
from launch_ros.descriptions    import ComposableNode
from aist_bringup.launch_common import (declare_launch_arguments,
                                        get_camera_props)


launch_arguments = [
    {
        'name':        'name',
        'default':     'depth_filter',
        'description': 'node name of the filter'
    },
    {
        'name':        'camera_name',
        'default':     'realsense',
        'description': 'name of the camera'
    },
    {
        'name':        'camera_type',
        'default':     'RealsenseCamera',
        'description': 'type of the camera'
    },
    {
        'name':        'param_file',
        'default':     PathJoinSubstitution(
                           [FindPackageShare('aist_depth_filter'), 'config',
                            'default.yaml']),
        'description': 'absolute path to YAML file for configuring detector'
    },
    {
        'name':        'external_container',
        'default':     'true',
        'description': 'use external container launched in advance',
        'choices':     ['true', 'false', 'True', 'False']
    },
    {
        'name':        'container',
        'default':     'cameras_container',
        'description': 'name of internal or external component container'
    },
    {
        'name':        'log_level',
        'default':     'info',
        'description': 'debug log level',
        'choices':     ['debug', 'info', 'warn', 'error', 'fatal']
    },
    {
        'name':        'output',
        'default':     'screen',
        'description': 'pipe node output',
        'choices':     ['screen', 'log', 'both']
    }
]


def launch_setup(context):
    camera_props = get_camera_props(LaunchConfiguration('camera_type')\
                                    .perform(context))
    remappings   = [('/camera_info',
                     [LaunchConfiguration('camera_name'), '/',
                      camera_props['cinfo_topic']]),
                    ('/depth',
                     [LaunchConfiguration('camera_name'), '/',
                      camera_props['depth_topic']]),
                    ('/image',
                     [LaunchConfiguration('camera_name'), '/',
                      camera_props['color_topic']])]
    if camera_props.get('normal_topic') is not None:
        remappings.append(('/normal',
                           [LaunchConfiguration('camera_name'), '/',
                            camera_props['normal_topic']]))
    # else:
    #     SetLaunchConfiguration('subscribe_normal', False)

    return [
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
                    name=LaunchConfiguration('name'),
                    package='aist_depth_filter',
                    plugin='aist_depth_filter::DepthFilter',
                    parameters=[LaunchConfiguration('param_file')],
                    remappings=remappings,
                    extra_arguments=[{'use_intra_process_comms': True}])]),
    ]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
