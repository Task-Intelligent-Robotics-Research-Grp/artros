from launch                            import LaunchDescription
from launch.actions                    import (DeclareLaunchArgument,
                                               IncludeLaunchDescription,
                                               OpaqueFunction,
                                               RegisterEventHandler)
from launch.conditions                 import IfCondition, UnlessCondition
from launch.event_handlers             import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions              import (Command, FindExecutable,
                                               LaunchConfiguration,
                                               ThisLaunchFileDir,
                                               PathJoinSubstitution,
                                               IfElseSubstitution)
from launch_ros.actions                import Node
from launch_ros.substitutions          import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


launch_arguments = [
    {'name':        'config',
     'default':     'a_bot',
     'description': 'configuration name of the scene'},
    {'name':        'scene',
     'default':     '',
     'description': 'name of the scene'},
    {'name':        'controllers_file',
     'default':     PathJoinSubstitution([ThisLaunchFileDir(), '..', 'config',
                                          'ur_controllers.yaml']),
     # 'default':     PathJoinSubstitution([FindPackageShare('ur_simulation_gz'),
     #                                      'config', 'ur_controllers.yaml']),
     'description': 'Absolute path to YAML file with the controllers configuration'},
    {'name':        'initial_joint_controller',
     'default':     'scaled_joint_trajectory_controller',
     'description': 'Robot controller to start'}]

def declare_launch_arguments(args):
    return [DeclareLaunchArgument(arg['name'],
                                  default_value=arg['default'],
                                  description=arg['description']) \
            for arg in args]

def launch_setup(context):
    robot_description_content \
        = Command([PathJoinSubstitution([FindExecutable(name='xacro')]),
                   ' ',
                   PathJoinSubstitution([FindPackageShare('aist_description'),
                                         'scenes', 'urdf',
                                         [LaunchConfiguration('config'),
                                          '_base_scene.urdf.xacro']]),
                   ' scene:=', LaunchConfiguration('scene'),
                   ' simulation_controllers:=',
                   LaunchConfiguration('controllers_file')])
    # print(robot_description_content.perform(context))

    # General arguments
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    return [Node(package='robot_state_publisher',
                 executable='robot_state_publisher',
                 output='both',
                 parameters=[{'use_sim_time': True,
                              'robot_description':
                              ParameterValue(robot_description_content,
                                             value_type=str)}]),
            Node(package='controller_manager',
                 executable='spawner',
                 arguments=['joint_state_broadcaster',
                            '--controller-manager', '/a_bot_controller_manager']),
            Node(package='controller_manager',
                 executable='spawner',
                 arguments=[initial_joint_controller,
                            '-c', '/a_bot_controller_manager']),
            Node(package='ros_gz_sim',
                 executable='create',
                 output='screen',
                 arguments=['-topic', '/robot_description',
                            '-name', LaunchConfiguration('config'),
                            '-allow_renaming', 'true']),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                          'launch', 'gz_sim.launch.py'])),
                launch_arguments={'gz_args': '-r -v 4 empty.sdf'}.items()),
            Node(package='ros_gz_bridge',
                 executable='parameter_bridge',
                 arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
                 output='screen')]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
