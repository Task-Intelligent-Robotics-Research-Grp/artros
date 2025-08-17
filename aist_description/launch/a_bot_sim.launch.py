from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    IfElseSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


launch_arguments = [
    {'name':        'config',
     'default':     'aist',
     'description': 'configuration name of the scene'},
    {'name':        'scene',
     'default':     '',
     'description': 'name of the scene'},
    {'name':        'sim',
     'default':     'false',
     'description': 'use setting of gazebo simulation if true'},
    {'name':        'rvizconfig',
     'default':     'display_scene.rviz',
     'description': 'path to configuration file of rviz'},
    {'name':        'joint_gui',
     'default':     'true',
     'description': 'launch joint_state_publisher if true'}]

def declare_launch_arguments(args):
    return [DeclareLaunchArgument(arg['name'],
                                  default_value=arg['default'],
                                  description=arg['description']) \
            for arg in args]

def launch_setup(context):
    urdf_path = PathJoinSubstitution([ThisLaunchFileDir(),
                                      '..', 'scenes', 'urdf',
                                      [LaunchConfiguration('config'),
                                       '_base_scene.urdf.xacro']])
    robot_description = ParameterValue(Command(['xacro ', urdf_path,
                                                ' scene:=',
                                                LaunchConfiguration('scene'),
                                                ' sim:=',
                                                LaunchConfiguration('sim')]),
                                       value_type=str)
    world_file = 'empty.sdf'
    gazebo_gui = 'true'

    # General arguments
    controllers_file = LaunchConfiguration("controllers_file")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    return [Node(package='robot_state_publisher',
                 executable='robot_state_publisher',
                 output='both',
                 parameters=[{'use_sim_time': True},
                             {'robot_description': robot_description}]),
            Node(package='controller_manager',
                 executable='spawner',
                 arguments=['joint_state_broadcaster',
                            '--controller-manager', '/controller_manager']),
            Node(package='ros_gz_sim',
                 executable='create',
                 output='screen',
                 arguments=['-string', robot_description,
                            '-name', 'ur', '-allow_renaming', 'true']),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                          'launch' 'gz_sim.launch.py'])),
                launch_arguments={'gz_args': '-r -v 4 empty.sdf'}.items()),
            Node(package='ros_gz_bridge',
                 executable='parameter_bridge',
                 arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
                 output='screen')]
    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(launch_rviz),
    )

    # There may be other controllers of the joints, but this is the initially-started one
    initial_joint_controller_spawner_started = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[initial_joint_controller, '-c', '/controller_manager'],
        condition=IfCondition(activate_joint_controller),
    )
    initial_joint_controller_spawner_stopped = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[initial_joint_controller, '-c', '/controller_manager', '--stopped'],
        condition=UnlessCondition(activate_joint_controller),
    )
