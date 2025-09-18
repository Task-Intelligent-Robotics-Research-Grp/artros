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
        'name':        'name',
        'default':     'precision_tool',
        'description': 'name of the precision tool'
    },
    {
        'name':        'usb_port',
        'default':     '/dev/ttyUSB1',
        'description': 'device name of the USB port'
    },
    {
        'name':        'baud_rate',
        'default':     '57600',
        'description': 'baud rate of the serial communication'
    },
    {
        'name':        'motor_id',
        'default':     '1',
        'description': 'ID of the Dynamixel motor'
    },
    {
        'name':        'container',
        'default':     'precision_tools_container',
        'description': 'name of the component container'
    },
    {
        'name':        'driver_ns',
        'default':     'precision_tools_driver',
        'description': 'name of the Dynamixel driver'
    },
    {
        'name':        'log_level',
        'default':     'info',
        'description': 'debug log level',
        'choices':     ['debug', 'info', 'warn', 'error', 'fatal']},
    {
        'name':        'output',
        'default':     'both',
        'description': 'pipe node output',
        'choices':     ['screen', 'log', 'both']
    }
]

def launch_setup(context):
    driver_param_file \
        = ParameterFile(PathJoinSubstitution(
                            [FindPackageShare('aist_fastening_tools'),
                             'config', 'dynamixel_driver.yaml']),
                        allow_substs=True)
    controller_param_file \
        = ParameterFile(PathJoinSubstitution(
                            [FindPackageShare('aist_fastening_tools'),
                             'config', 'precision_tool_controller.yaml']),
                        allow_substs=True)
    instantiate_file(context,
                     PathJoinSubstitution(
                                 [FindPackageShare('aist_fastening_tools'),
                                  'config',
                                  'precision_tool_dynamixel_info.yaml']),
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
                    name=[LaunchConfiguration('name'), '_controller'],
                    package='aist_fastening_tools',
                    plugin='aist_fastening_tools::PrecisionGripperController',
                    parameters=[controller_param_file],
                    extra_arguments=[{'use_intra_process_comms': True}])]
        )
    ]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
