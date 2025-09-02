from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import (
    AndSubstitution,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


launch_arguments = [
    {'name':        'initial_controller',
     'default':     'scaled_joint_trajectory_controller',
     'description': 'Initially loaded active controller',
     'choices':     ['scaled_joint_trajectory_controller',
                     'joint_trajectory_controller',
                     'forward_velocity_controller',
                     'forward_position_controller',
                     'freedrive_mode_controller',
                     'passthrough_trajectory_controller']},
    {'name':        'consistent_controllers',
     'default':     ['joint_state_broadcaster',
                     'io_and_status_controller',
                     'speed_scaling_state_broadcaster',
                     'force_torque_sensor_boradcaster',
                     'tcp_pose_broadcaster',
                     'ur_configuration_controller'],
     'description': 'Persistenly loaded controllers'},
    {'name':        'inactive_controllers',
     'default':     ['joint_trajectory_controller',
                     'forward_velocity_controller',
                     'forward_position_controller',
                     'force_mode_controller',
                     'passthrough_trajectory_controller',
                     'freedrive_mode_controller',
                     'tool_contact_controller'],
     'description': 'Initially loaded but inactive controllers'},
]

def declare_launch_arguments(args):
    return [DeclareLaunchArgument(arg['name'],
                                  default_value=arg.get('default'),
                                  description=arg.get('description'),
                                  choices=arg.get('choices')) \
            for arg in args]

def launch_setup(context):
    # Initialize Arguments
    # General arguments
    controllers_file = LaunchConfiguration('controllers_file')
    initial_joint_controller = LaunchConfiguration('initial_joint_controller')
    headless_mode = LaunchConfiguration('headless_mode')
    use_tool_communication = LaunchConfiguration('use_tool_communication')
    tool_device_name = LaunchConfiguration('tool_device_name')
    tool_tcp_port = LaunchConfiguration('tool_tcp_port')

    actions = [
        Node(package='controller_manager',
             executable='ros2_control_node',
             parameters=[
                 LaunchConfiguration('update_rate_config_file'),
                 ParameterFile(controllers_file, allow_substs=True)],
             output='screen'),
        Node(package='ur_robot_driver',
             executable='dashboard_client',
             name='dashboard_client',
             output='screen',
             emulate_tty=True,
             parameters=[{'robot_ip': robot_ip}]),
        Node(package='ur_robot_driver',
             executable='robot_state_helper',
             name='ur_robot_state_helper',
             output='screen',
             parameters=[{'headless_mode': headless_mode,
                          'robot_ip': robot_ip}]),
        Node(package='ur_robot_driver',
             condition=IfCondition(use_tool_communication),
             executable='tool_communication.py',
             name='ur_tool_comm',
             output='screen',
             parameters=[{'robot_ip': robot_ip,
                          'tcp_port': tool_tcp_port,
                          'device_name': tool_device_name}]),
        Node(package='ur_robot_driver',
             executable='urscript_interface',
             parameters=[{'robot_ip': robot_ip}],
             output='screen'),
        Node(package='ur_robot_driver',
             executable='controller_stopper_node',
             name='controller_stopper',
             output='screen',
             emulate_tty=True,
             parameters=[{'headless_mode': headless_mode},
                         {'joint_controller_active':
                          activate_joint_controller},
                         {'consistent_controllers':
                          LaunchConfiguration('consistent_controllers')}])]

    # Spawn controllers
    def controller_spawner(controllers, active=True):
        inactive_flags = ['--inactive'] if not active else []
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                '--controller-manager',
                '/controller_manager',
                '--controller-manager-timeout',
                controller_spawner_timeout,
            ]
            + inactive_flags
            + controllers,
        )

    controllers_active = [
        'joint_state_broadcaster',
        'io_and_status_controller',
        'speed_scaling_state_broadcaster',
        'force_torque_sensor_broadcaster',
        'tcp_pose_broadcaster',
        'ur_configuration_controller',
    ]
    controllers_inactive = [
        'scaled_joint_trajectory_controller',
        'joint_trajectory_controller',
        'forward_velocity_controller',
        'forward_position_controller',
        'force_mode_controller',
        'passthrough_trajectory_controller',
        'freedrive_mode_controller',
        'tool_contact_controller',
    ]



def generate_launch_description():
    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            'ur_type',
            description='Type/series of used UR robot.',
            choices=[
                'ur3',
                'ur3e',
                'ur5',
                'ur5e',
                'ur7e',
                'ur10',
                'ur10e',
                'ur12e',
                'ur16e',
                'ur15',
                'ur20',
                'ur30',
            ],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_ip', description='IP address by which the robot can be reached.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'safety_limits',
            default_value='true',
            description='Enables the safety limits controller if true.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'safety_pos_margin',
            default_value='0.15',
            description='The margin to lower and upper limits in the safety controller.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'safety_k_position',
            default_value='20',
            description='k-position factor in the safety controller.',
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            'controllers_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('ur_robot_driver'), 'config', 'ur_controllers.yaml']
            ),
            description='YAML file with the controllers configuration.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_launchfile',
            default_value=PathJoinSubstitution(
                [FindPackageShare('ur_robot_driver'), 'launch', 'ur_rsp.launch.py']
            ),
            description='Launchfile (absolute path) providing the description. '
            'The launchfile has to start a robot_state_publisher node that '
            'publishes the description topic.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'tf_prefix',
            default_value='',
            description='tf_prefix of the joint names, useful for '
            'multi-robot setup. If changed, also joint names in the controllers' configuration '
            'have to be updated.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='false',
            description='Start robot with mock hardware mirroring command to its states.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'mock_sensor_commands',
            default_value='false',
            description='Enable mock command interfaces for sensors used for simple simulations. '
            'Used only if 'use_mock_hardware' parameter is true.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'headless_mode',
            default_value='false',
            description='Enable headless mode for robot control',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'controller_spawner_timeout',
            default_value='10',
            description='Timeout used when spawning controllers.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'initial_joint_controller',
            default_value='scaled_joint_trajectory_controller',
            choices=[
                'scaled_joint_trajectory_controller',
                'joint_trajectory_controller',
                'forward_velocity_controller',
                'forward_position_controller',
                'freedrive_mode_controller',
                'passthrough_trajectory_controller',
            ],
            description='Initially loaded robot controller.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'activate_joint_controller',
            default_value='true',
            description='Activate loaded joint controller.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument('launch_rviz', default_value='true', description='Launch RViz?')
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('ur_description'), 'rviz', 'view_robot.rviz']
            ),
            description='RViz config file (absolute path) to use when launching rviz.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'launch_dashboard_client',
            default_value='true',
            description='Launch Dashboard Client?',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_tool_communication',
            default_value='false',
            description='Only available for e series!',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'tool_parity',
            default_value='0',
            description='Parity configuration for serial communication. Only effective, if '
            'use_tool_communication is set to True.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'tool_baud_rate',
            default_value='115200',
            description='Baud rate configuration for serial communication. Only effective, if '
            'use_tool_communication is set to True.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'tool_stop_bits',
            default_value='1',
            description='Stop bits configuration for serial communication. Only effective, if '
            'use_tool_communication is set to True.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'tool_rx_idle_chars',
            default_value='1.5',
            description='RX idle chars configuration for serial communication. Only effective, '
            'if use_tool_communication is set to True.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'tool_tx_idle_chars',
            default_value='3.5',
            description='TX idle chars configuration for serial communication. Only effective, '
            'if use_tool_communication is set to True.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'tool_device_name',
            default_value='/tmp/ttyUR',
            description='File descriptor that will be generated for the tool communication device. '
            'The user has be be allowed to write to this location. '
            'Only effective, if use_tool_communication is set to True.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'tool_tcp_port',
            default_value='54321',
            description='Remote port that will be used for bridging the tool's serial device. '
            'Only effective, if use_tool_communication is set to True.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'tool_voltage',
            default_value='0',  # 0 being a conservative value that won't destroy anything
            description='Tool voltage that will be setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'reverse_ip',
            default_value='0.0.0.0',
            description='IP that will be used for the robot controller to communicate back to the driver.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'script_command_port',
            default_value='50004',
            description='Port that will be opened to forward URScript commands to the robot.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'reverse_port',
            default_value='50001',
            description='Port that will be opened to send cyclic instructions from the driver to the robot controller.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'script_sender_port',
            default_value='50002',
            description='The driver will offer an interface to query the external_control URScript on this port.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'trajectory_port',
            default_value='50003',
            description='Port that will be opened for trajectory control.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name='update_rate_config_file',
            default_value=[
                PathJoinSubstitution(
                    [
                        FindPackageShare('ur_robot_driver'),
                        'config',
                    ]
                ),
                '/',
                LaunchConfiguration('ur_type'),
                '_update_rate.yaml',
            ],
        )
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
