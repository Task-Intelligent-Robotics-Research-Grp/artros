import os, yaml, shutil
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
from aist_bringup.launch_common        import (declare_launch_arguments,
                                               load_config, get_arm_props,
                                               get_gripper_props,
                                               get_camera_props,
                                               instantiate_config_file)

launch_arguments = [
    {
        'name':        'config',
        'default':     'aist',
        'description': 'Name of the hardware configuration'
    },
    {
        'name':        'scene',
        'default':     '',
        'description': 'Name of the scene'
    },
    {
        'name':        'sim',
        'default':     'false',
        'description': 'Launch gz if true',
        'choices':     ['true', 'false', 'True', 'False']
    },
]


def launch_setup(context):
    config = load_config(context)

    # Instantiate controller configuration files for each arm.
    arm_controllers_files = []
    update_rate = 0
    for arm_name, arm_config in config['arms'].items():
        arm_props = get_arm_props(arm_config['type'])
        if arm_props['update_rate'] > update_rate:
            update_rate = arm_props['update_rate']
            SetLaunchConfiguration('update_rate',
                                   str(update_rate)).execute(context)
        tf_prefix = arm_name + '_'
        SetLaunchConfiguration('tf_prefix', tf_prefix).execute(context)
        arm_controllers_files.append(
            instantiate_config_file(
                context,
                IfElseSubstitution(LaunchConfiguration('sim'),
                                   arm_props['gz_controllers_config_file'],
                                   arm_props['controllers_config_file']),
                '/tmp/' + tf_prefix + 'controllers.yaml'))

    # Instantiate controller configuration files for each gripper.
    grippers = {gripper_name: gripper_config for gripper_name, gripper_config
                in config['grippers'].items()
                if get_gripper_props(
                        gripper_config['type']).get(
                            'gz_controllers_config_file')}
    gripper_controllers_files = []
    for gripper_name, gripper_config in grippers.items():
        gripper_props = get_gripper_props(gripper_config['type'])
        tf_prefix = gripper_name + '_'
        SetLaunchConfiguration('tf_prefix', tf_prefix).execute(context)
        gripper_controllers_files.append(
            instantiate_config_file(context,
                                    gripper_props['gz_controllers_config_file'],
                                    '/tmp/' + tf_prefix + 'controllers.yaml'))

    # Setup a command for loading the URDF describing arms and environment.
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
                     parameters=[{'config_file': '/tmp/camera_bridge.yaml'}],
                     output='screen')]),
        GroupAction(
            condition=UnlessCondition(LaunchConfiguration('sim')),
            actions=[
                Node(package='controller_manager',
                     executable='ros2_control_node',
                     parameters=arm_controllers_files \
                               +gripper_controllers_files,
                     output='screen')])]

    actions += [
        Node(package='controller_manager',
             executable='spawner',
             arguments=[arm_name + '_joint_state_broadcaster',
                        arm_config['initial_controller'],
                        '--switch-timeout', '30'])
        for arm_name, arm_config in config['arms'].items()]

    actions += [
        Node(package='controller_manager',
             executable='spawner',
             arguments=[gripper_name + '_joint_state_broadcaster',
                        gripper_name + '_controller',
                        '--switch-timeout', '30'])
        for gripper_name in grippers.keys()]

    return actions

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
