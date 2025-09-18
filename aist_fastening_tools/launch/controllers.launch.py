from launch                            import LaunchDescription
from launch.actions                    import OpaqueFunction
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
        'default':     ['screw_tool_m3', 'screw_tool_m4'],
        'description': 'list of tool names'
    },
    {
        'name':        'tool_types',
        'default':     ['ScrewTool', 'ScrewTool'],
        'description': 'list of tool names'
    },
    {
        'name':        'motor_ids',
        'default':     ['2', '3']
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
        'dxlinfo_template':    PathJoinSubstitution(
                                   [FindPackageShare('aist_fastening_tools'),
                                    'config',
                                    'precision_tool_dynamixel_info.yaml']),
        'controller_template': PathJoinSubstitution(
                                   [FindPackageShare('aist_fastening_tools'),
                                    'config',
                                    'precision_tool_controller.yaml']),
        'controller_suffix':   '_controller',
        'plugin':              'aist_fastening_tools::PrecisionToolController',
    },
    'ScrewTool':
    {
        'dxlinfo_template':    PathJoinSubstitution(
                                   [FindPackageShare('aist_fastening_tools'),
                                    'config',
                                    'screw_tool_dynamixel_info.yaml']),
        'controller_template': PathJoinSubstitution(
                                   [FindPackageShare('aist_fastening_tools'),
                                    'config',
                                    'screw_tool_fastening_controller.yaml']),
        'controller_suffix':   '_fastening_controller',
        'plugin':              'aist_fastening_tools::ScrewTooController',
    },
}

def launch_setup(context):
    tool_controllers = []
    for tool_name, tool_type, motor_id \
            in zip(LaunchConfiguration('tool_names').perofrm(context),
                   LaunchConfiguration('tool_types').perofrm(context),
                   LaunchConfiguration('motor_ids').perofrm(context)):
        tools_props =TOOL_PROPS[tool_type]
        SetLaunchConfiguration('name',    tool_name).execute(context)
        SetLaunchConfiguration('motor_id', motor_id).execute(context)

        controller_param_file \
            = ParameterFile(tool_props['controller_template'],
                            allow_substs=True)

                   driver_param_file \
        = ParameterFile(PathJoinSubstitution(
                            [FindPackageShare('aist_fastening_tools'),
                             'config', 'dynamixel_driver.yaml']),
                        allow_substs=True)
    instantiate_file(context,
                     PathJoinSubstitution(
                                 [FindPackageShare('aist_fastening_tools'),
                                  'config', 'screw_tool_dynamixel_info.yaml']),
                     '/tmp/' \
                     + LaunchConfiguration('driver_ns').perform(context) \
                     + '_dynamixel_info.yaml')
    return [
        Node(name=LaunchConfiguration('container'),
             package='rclcpp_components',
             executable='component_container_mt',
             output=LaunchConfiguration('output'),
             arguments=['--ros-args', '--log-level',
                        LaunchConfiguration('log_level')]),
        LoadComposableNodes(
            target_container=LaunchConfiguration('container'),
            composable_node_descriptions=[
                ComposableNode(
                    name=LaunchConfiguration('driver_ns'),
                    package='dynamixel_workbench_controllers',
                    plugin='dynamixel_workbench_controllers::DynamixelController',
                    parameters=[driver_param_file],
                    extra_arguments=[{'use_intra_process_comms': True}]),
                ComposableNode(
                    name=[LaunchConfiguration('name'),
                          '_fastening_controller'],
                    package='aist_fastening_tools',
                    plugin='aist_fastening_tools::ScrewToolController',
                    parameters=[controller_param_file],
                    extra_arguments=[{'use_intra_process_comms': True}])
            ]
        )
    ]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
