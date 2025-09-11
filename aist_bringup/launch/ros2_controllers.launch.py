from launch                            import LaunchDescription
from launch.actions                    import (SetLaunchConfiguration,
                                               IncludeLaunchDescription,
                                               OpaqueFunction,
                                               GroupAction,
                                               RegisterEventHandler)
from launch.conditions                 import IfCondition, UnlessCondition
from launch.substitutions              import (Command, FindExecutable,
                                               LaunchConfiguration,
                                               PathJoinSubstitution,
                                               IfElseSubstitution)
from launch.event_handlers             import OnProcessStart
from launch_ros.actions                import Node
from launch_ros.substitutions          import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from aist_bringup.launch_common        import (declare_launch_arguments,
                                               load_config, get_arm_props,
                                               get_gripper_props,
                                               instantiate_file)

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
    sim    = LaunchConfiguration('sim').perform(context) in ('true', 'True')

    # Instantiate controller configuration files for each arm.
    controllers_files = []
    update_rate = 0
    for arm_name, arm_config in config['arms'].items():
        arm_props = get_arm_props(arm_config['type'])
        if arm_props['update_rate'] > update_rate:
            update_rate = arm_props['update_rate']
            SetLaunchConfiguration('update_rate',
                                   str(update_rate)).execute(context)
        template = arm_props.get('gz_controllers_template') if sim else \
                   arm_props.get('controllers_template')
        if template is not None:
            tf_prefix = arm_name + '_'
            SetLaunchConfiguration('tf_prefix', tf_prefix).execute(context)
            controllers_files.append(
                instantiate_file(context, template,
                                 '/tmp/' + tf_prefix + 'controllers.yaml'))

    # Instantiate controller configuration files for each gripper.
    gripper_names = []
    for gripper_name, gripper_config in config['grippers'].items():
        gripper_props = get_gripper_props(gripper_config['type'])
        template = gripper_props.get('gz_controllers_template') if sim else \
                   gripper_props.get('controllers_template')
        if template is not None:
            tf_prefix = gripper_name + '_'
            SetLaunchConfiguration('tf_prefix', tf_prefix).execute(context)
            controllers_files.append(
                instantiate_file(context, template,
                                 '/tmp/' + tf_prefix + 'controllers.yaml'))
            gripper_names.append(gripper_name)

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

    rsp_node = Node(package='robot_state_publisher',
                    executable='robot_state_publisher',
                    parameters=[{'use_sim_time': LaunchConfiguration('sim')},
                                {'robot_description':
                                 ParameterValue(robot_description_content,
                                                value_type=str)}],
                    output='screen')
    actions = [
        rsp_node,
        RegisterEventHandler(
            OnProcessStart(
                target_action=rsp_node,
                on_start=[
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
                              ' gz-physics-bullet-featherstone-plugin'])])]),
            condition=IfCondition(LaunchConfiguration('sim'))),
        RegisterEventHandler(
            OnProcessStart(
                target_action=rsp_node,
                on_start=[
                    Node(package='controller_manager',
                         executable='ros2_control_node',
                         parameters=controllers_files,
                         output='screen')]),
            condition=UnlessCondition(LaunchConfiguration('sim')))]

    active_controllers   = ['joint_state_broadcaster']
    inactive_controllers = []
    for arm_name, arm_config in config['arms'].items():
        active_controllers.append(arm_config['initial_controller'])
        active_controllers   += arm_config.get('consistent_controllers', [])
        inactive_controllers += arm_config.get('inactive_controllers', [])
        if not sim:
            active_controllers \
                += arm_config.get('extra_consistent_controllers', [])
            inactive_controllers \
                += arm_config.get('extra_inactive_controllers', [])
    for gripper_name in gripper_names:
        active_controllers.append(gripper_name + '_controller')

    actions += [
        Node(package='controller_manager',
             executable='spawner',
             arguments=['--switch-timeout', '30'] + active_controllers),
        Node(package='controller_manager',
             executable='spawner',
             arguments=['--switch-timeout', '30',
                        '--inactive'] + inactive_controllers)]

    return actions

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
