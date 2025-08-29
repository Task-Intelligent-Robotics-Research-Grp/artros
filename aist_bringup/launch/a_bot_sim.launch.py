import os
from launch                            import LaunchDescription
from launch.actions                    import (SetLaunchConfiguration,
                                               DeclareLaunchArgument,
                                               IncludeLaunchDescription,
                                               OpaqueFunction,
                                               RegisterEventHandler,
                                               GroupAction)
from launch.conditions                 import IfCondition, UnlessCondition
from launch.event_handlers             import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions              import (Command, FindExecutable,
                                               LaunchConfiguration,
                                               ThisLaunchFileDir,
                                               PathJoinSubstitution,
                                               IfElseSubstitution)
from launch_ros.actions                import Node, PushROSNamespace
from launch_ros.substitutions          import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile


launch_arguments = [
    {'name':        'config',
     'default':     'a_bot',
     'description': 'Name of the hardware configuration'},
    {'name':        'scene',
     'default':     '',
     'description': 'Name of the scene'},
    {'name':        'controllers_file',
     'default':     PathJoinSubstitution([ThisLaunchFileDir(), '..', 'config',
                                          'ur_controllers.yaml']),
     'description': 'Absolute path to YAML file configuring controllers'},
    {'name':        'activate_joint_controller',
     'default':     'true',
     'description': 'Enable headless mode for robot cotnrol'},
    {'name':        'initial_joint_controller',
     'default':     'scaled_joint_trajectory_controller',
     'description': 'Robot controller to start'},
    {'name':        'joint_state_pub_rate',
     'default':     '50.0',
     'description': 'Rate of publishing joint state'},
    {'name':        'vis',
     'default':     'true',
     'description': 'Launch rviz2 if true',
     'choices':     ['true', 'false']},
    {'name':        'rviz_config_file',
     'default':     PathJoinSubstitution([ThisLaunchFileDir(),
                                          [LaunchConfiguration('config'),
                                           '.rviz']])},
    {'name':        'tf_prefix',
     'default':     'a_bot_'}]


def declare_launch_arguments(args):
    return [DeclareLaunchArgument(arg['name'],
                                  default_value=arg.get('default'),
                                  description=arg.get('description'),
                                  choices=arg.get('choices')) \
            for arg in args]

def set_configurable_parameters(args):
    return {arg['name']: LaunchConfiguration(arg['name']) for arg in args}

def create_substituted_controllers_file(context, robot_name):
    SetLaunchConfiguration('tf_prefix', value=robot_name + '_')
    print(LaunchConfiguration('tf_prefix').perform(context))

    # We must extend lifetime of the ParameterFile object by keeping it
    # in a variable because the temporary file created by evaluation
    # will be erased by the destructor of ParameterFile.
    parameter_file = ParameterFile(LaunchConfiguration('controllers_file'),
                                   allow_substs=True)
    # Rename the created file to prevent from being erased.
    os.rename(parameter_file.evaluate(context),
              '/tmp/' + robot_name + '_controllers.yaml')

def launch_setup(context):
    robot_names = ['a_bot']

    print('*** OK0')
    for robot_name in robot_names:
        print('*** OK1')
        create_substituted_controllers_file(context, robot_name)
        print('*** OK2')
    robot_description_content \
        = Command([FindExecutable(name='xacro'),
                   ' ',
                   PathJoinSubstitution([FindPackageShare('aist_description'),
                                         'scenes', 'urdf',
                                         [LaunchConfiguration('config'),
                                          '_base_scene.urdf.xacro']]),
                   ' scene:=', LaunchConfiguration('scene'),
                   ' sim:=', 'true'])
    print('*** OK3')

    actions = [
        Node(package="joint_state_publisher",
             executable="joint_state_publisher",
             parameters=[{"rate": LaunchConfiguration('joint_state_pub_rate')},
                         {"source_list":
                          [[robot_name, "/joint_states"] \
                           for robot_name in robot_names]}],
             output="log"),
        Node(package="robot_state_publisher",
             executable="robot_state_publisher",
             parameters=[{"use_sim_time": True},
                         {"robot_description":
                          ParameterValue(robot_description_content,
                                         value_type=str)}],
             output="log")]

    for robot_name in robot_names:
        actions += [
            PushROSNamespace(robot_name),
            Node(package='controller_manager',
                 executable='spawner',
                 arguments=['joint_state_broadcaster',
                            '-c', 'controller_manager'],
                 output='screen'),
            Node(package='controller_manager',
                 executable='spawner',
                 arguments=[LaunchConfiguration('initial_joint_controller'),
                            '-c', 'controller_manager'],
                 output='screen')]
    print('*** OK4')

    actions += [
        PushROSNamespace('/'),
        Node(package='ros_gz_sim',
             executable='create',
             arguments=['-topic', '/robot_description',
                        '-name',  [LaunchConfiguration('config'),
                                   '_base_scene'],
                        '-allow_renaming', 'true'],
             output='log'),
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch',
                                  'gz_sim.launch.py']),
            launch_arguments=[('gz_args', [' -r -v 4 ', 'empty.sdf'])]),
        Node(package='ros_gz_bridge',
             executable='parameter_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
             output="log"),
        Node(package="rviz2",
             executable="rviz2",
             name="rviz2",
             arguments=["-d", LaunchConfiguration("rviz_config_file")],
             output="log")]

    return actions

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
