import os, yaml
from launch                            import LaunchDescription
from launch.actions                    import (SetLaunchConfiguration,
                                               DeclareLaunchArgument,
                                               IncludeLaunchDescription,
                                               OpaqueFunction,
                                               GroupAction)
from launch.conditions                 import IfCondition, UnlessCondition
from launch.substitutions              import (Command, FindExecutable,
                                               LaunchConfiguration,
                                               ThisLaunchFileDir,
                                               PathJoinSubstitution,
                                               IfElseSubstitution)
from launch_ros.actions                import Node
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
     'default':     IfElseSubstitution(LaunchConfiguration('sim'),
                                       'joint_trajectory_controller',
                                       'scaled_joint_trajectory_controller'),
     'description': 'Robot controller to start'},
    {'name':        'update_rate',
     'default':     '500',
     'description': 'Update rate for controller manager'},
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

def instantiate_controllers_file(context, template_file, tf_prefix):
    SetLaunchConfiguration('tf_prefix', value=tf_prefix).execute(context)
    # We must extend lifetime of the ParameterFile object by keeping it
    # in a variable or the temporary file created by evaluating it
    # will be immediately erased by the destructor of ParameterFile.
    parameter_file = ParameterFile(template_file, allow_substs=True)
    # Rename the created file to prevent from being erased.
    instantiated_file = '/tmp/' + tf_prefix + 'controllers.yaml'
    os.rename(parameter_file.evaluate(context), instantiated_file)
    return instantiated_file

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
    gripper_names = [gripper_name for gripper_name in config['grippers'].keys()
                     if config['grippers'][gripper_name]['type'] == 'RobotiqGripper']

    # Instantiate controller files from templates for each robot/gripper.
    robot_controllers_files = [
        instantiate_controllers_file(context,
                                     LaunchConfiguration('controllers_file'),
                                     robot_name + '_')
        for robot_name in robot_names]
    gripper_controllers_files = [
        instantiate_controllers_file(context,
                                     LaunchConfiguration(
                                         'gripper_controllers_file'),
                                     gripper_name + '_')
        for gripper_name in gripper_names]

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
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             parameters=[{'use_sim_time': LaunchConfiguration('sim')},
                         {'robot_description':
                          ParameterValue(robot_description_content,
                                         value_type=str)}],
             output='screen'),
        GroupAction(
            condition=IfCondition(LaunchConfiguration('sim')),
            actions=[
                Node(package='ros_gz_sim',
                     executable='create',
                     arguments=['-topic', 'robot_description'],
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
                     output='screen')]),
        GroupAction(
            condition=UnlessCondition(LaunchConfiguration('sim')),
            actions=[
                Node(package='controller_manager',
                     executable='ros2_control_node',
                     parameters=robot_controllers_files \
                               +gripper_controllers_files,
                         # ParameterFile(LaunchConfiguration('controllers_file'),
                         #               allow_substs=True)],
                     output='screen')]),
        Node(condition=IfCondition(LaunchConfiguration('vis')),
             package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['-d', LaunchConfiguration('rviz_config_file')],
             output='screen')]

    actions += [
        Node(package='controller_manager',
             executable='spawner',
             arguments=[robot_name + '_joint_state_broadcaster',
                        [robot_name,  '_',
                         LaunchConfiguration('initial_joint_controller')],
                        '--switch-timeout', '30'])
        for robot_name in robot_names]

    actions += [
        Node(package='controller_manager',
             executable='spawner',
             arguments=[gripper_name + '_joint_state_broadcaster',
                        gripper_name + '_controller',
                        '--switch-timeout', '30'])
        for gripper_name in gripper_names]

    return actions

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
