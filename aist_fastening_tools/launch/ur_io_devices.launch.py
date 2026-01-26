from launch                            import LaunchDescription
from launch.actions                    import OpaqueFunction
from launch.substitutions              import (LaunchConfiguration,
                                               PathJoinSubstitution)
from launch_ros.actions                import Node, LoadComposableNodes
from launch_ros.descriptions           import ComposableNode
from launch_ros.substitutions          import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile
from aist_bringup.launch_common        import declare_launch_arguments

launch_arguments = [
    {
        'name':        'param_file',
        'default':     PathJoinSubstitution(
                           [FindPackageShare('aist_fastening_tools'),
                            'config', 'default.yaml']),
        'description': 'absolute path to configuration file'
    },
    {
        'name':        'device_names',
        'default':     'screw_tool_m3,screw_tool_m4,suction_tool,base_fixture',
        'description': 'list of tool names'
    },
    {
        'name':        'container',
        'default':     'suction_tools_container',
        'description': 'name of the component container'
    },
    {
        'name':        'driver_ns',
        'default':     'b_bot_io_and_status_controller',
        'description': 'namespace of the IO controller of the UR arm'
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
    composable_nodes = []
    for device_name \
            in LaunchConfiguration('device_names').perform(context).split(','):
        composable_nodes.append(
            ComposableNode(
                name=device_name + '_controller',
                package='aist_fastening_tools',
                plugin='aist_fastening_tools::SuctionToolController',
                parameters=[
                    {'driver_ns': LaunchConfiguration('driver_ns')},
                    ParameterFile(LaunchConfiguration('param_file'))
                ],
                extra_arguments=[{'use_intra_process_comms': True}]))
    return [
        Node(name=LaunchConfiguration('container'),
             package='rclcpp_components',
             executable='component_container_mt',
             output=LaunchConfiguration('output'),
             arguments=['--ros-args', '--log-level',
                        LaunchConfiguration('log_level')]),
        LoadComposableNodes(
            target_container=LaunchConfiguration('container'),
            composable_node_descriptions=composable_nodes)
    ]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
