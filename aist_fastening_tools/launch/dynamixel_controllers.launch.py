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
                                               instantiate_file)

launch_arguments = [
    {
        'name':        'tool_names',
        'default':     'screw_tool_m3_fastening,screw_tool_m4_fastening',
        'description': 'list of tool names'
    },
    {
        'name':        'tool_types',
        'default':     'ScrewTool,ScrewTool',
        'description': 'list of tool names'
    },
    {
        'name':        'motor_ids',
        'default':     '2,3',
        'description': 'list of IDs of the Dynamixel motor'
    },
    {
        'name':        'usb_port',
        'default':     '/dev/ttyUSB0',
        'description': 'device name of the USB port'
    },
    {
        'name':        'baud_rate',
        'default':     '1000000',
        'description': 'baud rate of the serial communication'
    },
    {
        'name':        'container',
        'default':     'screw_tools_container',
        'description': 'name of the component container'
    },
    {
        'name':        'driver_ns',
        'default':     'screw_tools_driver',
        'description': 'name of the Dynamixel driver'
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

TOOL_PROPS = {
    'PrecisionTool':
    {
        'plugin':              'aist_fastening_tools::PrecisionToolController',
        'dxlinfo_template':    PathJoinSubstitution(
                                   [FindPackageShare('aist_fastening_tools'),
                                    'config',
                                    'precision_tool_dynamixel_info.yaml']),
        'controller_template': PathJoinSubstitution(
                                   [FindPackageShare('aist_fastening_tools'),
                                    'config',
                                    'precision_tool_controller.yaml']),
    },
    'ScrewTool':
    {
        'plugin':              'aist_fastening_tools::ScrewToolController',
        'dxlinfo_template':    PathJoinSubstitution(
                                   [FindPackageShare('aist_fastening_tools'),
                                    'config',
                                    'screw_tool_dynamixel_info.yaml']),
        'controller_template': PathJoinSubstitution(
                                   [FindPackageShare('aist_fastening_tools'),
                                    'config',
                                    'screw_tool_controller.yaml']),
    },
}

def launch_setup(context):
    actions = [
        Node(name=LaunchConfiguration('container'),
             package='rclcpp_components',
             executable='component_container_mt',
             output=LaunchConfiguration('output'),
             arguments=['--ros-args', '--log-level',
                        LaunchConfiguration('log_level')]),
    ]

    append = False
    for tool_name, tool_type, motor_id \
          in zip(LaunchConfiguration('tool_names').perform(context).split(','),
                 LaunchConfiguration('tool_types').perform(context).split(','),
                 LaunchConfiguration('motor_ids').perform(context).split(',')):
        tool_props = TOOL_PROPS[tool_type]
        actions += [
            SetLaunchConfiguration('name', tool_name),
            SetLaunchConfiguration('motor_id', motor_id),
            LoadComposableNodes(
                target_container=LaunchConfiguration('container'),
                composable_node_descriptions=[
                    ComposableNode(
                        name=tool_name + '_controller',
                        package='aist_fastening_tools',
                        plugin=tool_props['plugin'],
                        parameters=[ParameterFile(
                                        tool_props['controller_template'],
                                        allow_substs=True)],
                        extra_arguments=[{'use_intra_process_comms': True}])])
        ]
        SetLaunchConfiguration('name', tool_name).execute(context)
        SetLaunchConfiguration('motor_id', motor_id).execute(context)
        instantiate_file(context, tool_props['dxlinfo_template'],
                         '/tmp/' \
                         + LaunchConfiguration('driver_ns').perform(context) \
                         + '_dynamixel_info.yaml', append=append)
        append = True

    actions.append(
        LoadComposableNodes(
            target_container=LaunchConfiguration('container'),
            composable_node_descriptions=[
                ComposableNode(
                    name=LaunchConfiguration('driver_ns'),
                    package='dynamixel_workbench_controllers',
                    plugin='dynamixel_workbench_controllers::DynamixelController',
                    parameters=[
                        ParameterFile(
                            PathJoinSubstitution([
                                FindPackageShare('aist_fastening_tools'),
                                'config', 'dynamixel_driver.yaml']),
                            allow_substs=True)],
                    extra_arguments=[{'use_intra_process_comms': True}])]))
    return actions

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
