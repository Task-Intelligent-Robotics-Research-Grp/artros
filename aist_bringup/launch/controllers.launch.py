import os, yaml
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
     'default':     'aist',
     'description': 'Name of the hardware configuration'},
    {'name':        'scene',
     'default':     '',
     'description': 'Name of the scene'},
    {'name':        'sim',
     'default':     'false',
     'description': 'Launch gz if true',
     'choices':     ['true', 'false']},
    {'name':        'controllers_file',
     'default':     PathJoinSubstitution([ThisLaunchFileDir(), '..', 'config',
                                          'ur_controllers.yaml']),
     'description': 'Absolute path to YAML file configuring controllers'},
    {'name':        'gripper_controllers_file',
     'default':     PathJoinSubstitution([ThisLaunchFileDir(), '..', 'config',
                                          'gripper_controllers.yaml']),
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
     'default':     LaunchConfiguration('sim'),
     'description': 'Launch rviz2 if true',
     'choices':     ['true', 'false']},
    {'name':        'rviz_config_file',
     'default':     PathJoinSubstitution([ThisLaunchFileDir(),
                                          [LaunchConfiguration('config'),
                                           '.rviz']])}]


def declare_launch_arguments(args):
    return [DeclareLaunchArgument(arg['name'],
                                  default_value=arg.get('default'),
                                  description=arg.get('description'),
                                  choices=arg.get('choices')) \
            for arg in args]

def create_substituted_controllers_file(context, file_name, tf_prefix):
    SetLaunchConfiguration('tf_prefix', value=tf_prefix).execute(context)
    # We must extend lifetime of the ParameterFile object by keeping it
    # in a variable because the temporary file created by evaluation
    # will be erased by the destructor of ParameterFile.
    parameter_file = ParameterFile(file_name, allow_substs=True)
    # Rename the created file to prevent from being erased.
    os.rename(parameter_file.evaluate(context),
              '/tmp/' + tf_prefix + 'controllers.yaml')

def launch_setup(context):
    bridge_file = PathJoinSubstitution([FindPackageShare('aist_bringup'),
                                        'config',
                                        [LaunchConfiguration('config'),
                                         '_ros_gz_bridge.yaml']])
    config_file = PathJoinSubstitution([FindPackageShare('aist_bringup'),
                                        'config',
                                        [LaunchConfiguration('config'),
                                         '_config.yaml']])
    with open(config_file.perform(context), 'r') as f:
        config = yaml.safe_load(f)

    robot_names   = config['robots'].keys()
    gripper_names = config['grippers'].keys()

    # Create controller files instatiated from the template for each robot.
    for robot_name in robot_names:
        create_substituted_controllers_file(
            context, LaunchConfiguration('controllers_file'), robot_name + '_')

    # Create controller files instatiated from the template for each gripper.
    for gripper_name in gripper_names:
        if config['grippers'][gripper_name]['type'] == 'RobotiqGripper':
            create_substituted_controllers_file(
                context, LaunchConfiguration('gripper_controllers_file'),
                gripper_name + '_')

    # Setup a command for loading the URDF describing robots and environment.
    robot_description_content \
        = Command([FindExecutable(name='xacro'),
                   ' ',
                   PathJoinSubstitution([FindPackageShare('aist_description'),
                                         'scenes', 'urdf',
                                         [LaunchConfiguration('config'),
                                          '_base_scene.urdf.xacro']]),
                   ' scene:=', LaunchConfiguration('scene'),
                   ' sim:=',   LaunchConfiguration('sim')])

    actions = [
        Node(package="joint_state_publisher",
             executable="joint_state_publisher",
             parameters=[{"rate": LaunchConfiguration('joint_state_pub_rate')},
                         {"source_list":
                          [[robot_name, "/joint_states"] \
                           for robot_name in robot_names]}],
             output="screen"),
        Node(package="robot_state_publisher",
             executable="robot_state_publisher",
             parameters=[{"use_sim_time": True},
                         {"robot_description":
                          ParameterValue(robot_description_content,
                                         value_type=str)}],
             output="screen"),
        GroupAction(
            condition=IfCondition(LaunchConfiguration('sim')),
            actions=[
                Node(package='ros_gz_sim',
                     executable='create',
                     arguments=['-topic', 'robot_description',
                                '-name',  [LaunchConfiguration('config'),
                                           '_base_scene'],
                                '-allow_renaming', 'true'],
                     output='screen'),
                IncludeLaunchDescription(
                    PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                          'launch', 'gz_sim.launch.py']),
                    launch_arguments=[
                        ('gz_args',
                         [' -r -v 4 empty.sdf',
                          ' --physics-engine',
                          ' gz-physics-bullet-featherstone-plugin'])]),
                Node(package='ros_gz_bridge',
                     executable='parameter_bridge',
                     parameters=[{'config_file': bridge_file}],
                     output="screen")]),
        Node(condition=IfCondition(LaunchConfiguration('vis')),
             package="rviz2",
             executable="rviz2",
             name="rviz2",
             arguments=["-d", LaunchConfiguration("rviz_config_file")],
             output="screen")]

    for robot_name in robot_names:
        actions.append(
            GroupAction(
                actions=[
                    PushROSNamespace(robot_name),
                    Node(package='controller_manager',
                         executable='spawner',
                         arguments=['joint_state_broadcaster',
                                    '-c', 'controller_manager']),
                    Node(package='controller_manager',
                         executable='spawner',
                         arguments=[
                             LaunchConfiguration('initial_joint_controller'),
                             '-c', 'controller_manager'])]))
    for gripper_name in gripper_names:
        if config['grippers'][gripper_name]['type'] == 'RobotiqGripper':
            actions.append(
                GroupAction(
                    actions=[
                        PushROSNamespace(gripper_name),
                        Node(package='controller_manager',
                             executable='spawner',
                             arguments=['joint_state_broadcaster',
                                        '-c', 'controller_manager']),
                        Node(package='controller_manager',
                             executable='spawner',
                             arguments=['gripper_controller',
                                        '-c', 'controller_manager'])]))

    return actions

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
