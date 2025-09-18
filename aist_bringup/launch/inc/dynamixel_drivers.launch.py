from launch                            import LaunchDescription
from launch.actions                    import (SetLaunchConfiguration,
                                               OpaqueFunction)
from launch.substitutions              import (LaunchConfiguration,
                                               PathJoinSubstitution)
from launch_ros.actions                import Node, LoadComposableNodes
from launch_ros.descriptions           import ComposableNode
from launch_ros.substitutions          import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile
from aist_bringup.launch_common        import (declare_launch_arguments,
                                               load_config)


launch_arguments = [
    {
        'name':        'config',
        'default':     'aist',
        'description': 'Name of the hardware configuration'
    },
    {
        'name':        'name',
        'default':     'screw_tools',
        'description': 'Name of the Dynamixel device group'
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

DEVICE_PROPS = {
    'PrecisionTool':
    {
        'controller_template': PathJoinSubstitution(
                                   [FindPackageShare('aist_fastening_tools'),
                                    'config',
                                    'precision_tool_controller.yaml']),
        'controller_suffix':   '_controller',
        'plugin':              'aist_fastening_tools::PrecisionToolController',
    },
    'ScrewTool':
    {
        'controller_template': PathJoinSubstitution(
                                   [FindPackageShare('aist_fastening_tools'),
                                    'config',
                                    'screw_tool_fastening_controller.yaml']),
        'controller_suffix':   '_fastening_controller',
        'plugin':              'aist_fastening_tools::ScrewTooController',
    },
}

def launch_setup(context):
    config       = load_config(context)
    tools_config = config['grippers'][LaunchConfiguration('name') \
                                      .perform(context)]

    SetLaunchConfiguration('driver_ns',
                           LaunchConfiguration('name')).execute(context)
    SetLaunchConfiguration('usb_port',
                           tools_config['usb_port']).execute(context)
    SetLaunchConfiguration('baud_rate',
                           tools_config['baud_rate']).execute(context)

    driver_param_file \
        = ParameterFile(PathJoinSubstitution(
                            [FindPackageShare('aist_fastening_tools'),
                             'config', 'dynamixel_driver.yaml']),
                        allow_substs=True)
    composable_nodes = [
        ComposableNode(
            name=[LaunchConfiguration('name'), '_driver'],
            package='dynamixel_workbench_controllers',
            plugin='dynamixel_workbench_controllers::DynamixelController',
            parameters=[driver_param_file],
            extra_arguments=[{'use_intra_process_comms': True}])]
    for device_name, device_config in tools_config['devices'].items():
        device_props = DEVICE_PROPS[device_config['type']]
        controller_param_file \
            = ParameterFile(device_props['controller_template'],
                            allow_substs=True)
        composable_nodes.append(
            ComposableNode(
                name=device_name + device_props['controller_suffix'],
                package='aist_fastening_tools',
                plugin=device_props['plugin'],
                parameters=[controller_param_file],
                extra_arguments=[{'use_intra_process_comms': True}]))

    return [
        Node(name=[LaunchConfiguration('name'), '_container'],
             package='rclcpp_components',
             executable='component_container_mt',
             output=LaunchConfiguration('output'),
             arguments=['--ros-args', '--log-level',
                        LaunchConfiguration('log_level')]),
        LoadComposableNodes(
            target_container=[LaunchConfiguration('name'), '_container'],
            composable_node_descriptions=composable_nodes)]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
