from launch                            import LaunchDescription
from launch.actions                    import (SetLaunchConfiguration,
                                               IncludeLaunchDescription,
                                               OpaqueFunction,
                                               GroupAction)
from launch.conditions                 import IfCondition, UnlessCondition
from launch.substitutions              import (Command, FindExecutable,
                                               LaunchConfiguration,
                                               PathJoinSubstitution,
                                               IfElseSubstitution)
from launch_ros.actions                import Node, PushROSNamespace
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

    # Create an action for launching robot_state_publisher from loaded URDF.
    robot_description = ParameterValue(
                            Command([FindExecutable(name='xacro'),
                                     ' ',
                                     PathJoinSubstitution(
                                         [FindPackageShare('aist_description'),
                                          'scenes', 'urdf',
                                          [LaunchConfiguration('config'),
                                           '_base_scene.urdf.xacro']]),
                                     ' scene:=', LaunchConfiguration('scene'),
                                     ' sim:=',   LaunchConfiguration('sim')]),
                            value_type=str)

    # Setup actions for launching nodes,
    actions = [
        Node(package='joint_state_publisher',
             executable='joint_state_publisher',
             parameters=[{'rate': LaunchConfiguration('update_rate'),
                          'source_list':
                          [robot_name + '/joint_states' \
                           for robot_name in config['arms']]}],
             output='screen'),
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             parameters=[{'use_sim_time':      LaunchConfiguration('sim')},
                         {'robot_description': robot_description}],
             output='screen'),
        GroupAction(
            actions=[
                Node(package='ros_gz_sim',
                     executable='create',
                     arguments=['-topic', 'robot_description'],
                     output='screen'),
                IncludeLaunchDescription(
                    PathJoinSubstitution(
                        [FindPackageShare('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py']),
                    launch_arguments=[
                        ('gz_args', [' -r -v 4 empty.sdf', ' --physics-engine',
                                     ' gz-physics-bullet-featherstone-plugin'])
                    ])
            ],
            condition=IfCondition(LaunchConfiguration('sim'))),
    ]

    # Instantiate controller configuration files for each arm.
    for arm_name, arm_config in config['arms'].items():
        arm_props = get_arm_props(arm_config['type'])
        SetLaunchConfiguration('update_rate',
                               str(arm_props['update_rate'])).execute(context)
        SetLaunchConfiguration('tf_prefix', arm_name + '_').execute(context)
        SetLaunchConfiguration('speed_scaling_interface_name',
                               IfElseSubstitution(
                                   LaunchConfiguration('sim'),
                                   '""',
                                   'speed_scaling/speed_scaling_factor')
                               ).execute(context)
        controllers_file \
            = instantiate_file(context, arm_props['controllers_template'],
                               '/tmp/' + arm_name + '_controllers.yaml')

        active_controllers   = [arm_config['initial_controller']] \
                             + arm_config.get('consistent_controllers', [])
        inactive_controllers = arm_config.get('inactive_controllers', [])
        if not sim:
            active_controllers \
                += arm_config.get('extra_consistent_controllers', [])
            inactive_controllers \
                += arm_config.get('extra_inactive_controllers', [])
        print(active_controllers)

        actions.append(
            GroupAction(
                actions=[
                    PushROSNamespace(arm_name),
                    # SetLaunchConfiguration('update_rate',
                    #                        str(arm_props['update_rate'])),
                    # SetLaunchConfiguration('tf_prefix', arm_name + '_'),
                    # SetLaunchConfiguration(
                    #     'speed_scaling_interface_name',
                    #     IfElseSubstitution(
                    #         LaunchConfiguration('sim'),
                    #         '""', 'speed_scaling/speed_scaling_factor')),
                    Node(condition=UnlessCondition(LaunchConfiguration('sim')),
                         package='controller_manager',
                         executable='ros2_control_node',
                         parameters=[controllers_file],
                         output='screen'),
                    Node(package='controller_manager',
                         executable='spawner',
                         arguments=[
                             '-c', 'controller_manager',
                             '--switch-timeout', '30'
                         ] + active_controllers),
                    Node(package='controller_manager',
                         executable='spawner',
                         arguments=[
                             '-c', 'controller_manager',
                             '--switch-timeout', '30',
                             '--inactive'
                         ] + inactive_controllers)
                ]
            ))

    # Instantiate controller configuration files for each gripper.
    gripper_controllers = ['joint_state_broadcaster']
    for gripper_name, gripper_config in config['grippers'].items():
        gripper_props = get_gripper_props(gripper_config['type'])
        template = gripper_props.get('gz_controllers_template') if sim else \
                   gripper_props.get('controllers_template')
        if template is not None:
            SetLaunchConfiguration('tf_prefix',
                                   gripper_name + '_').execute(context)
            instantiate_file(context, template,
                             '/tmp/' + gripper_name + '_controllers.yaml')
            gripper_controllers.append(gripper_name + '_controller')

    # actions.append(
    #     Node(package='controller_manager',
    #          executable='spawner',
    #          arguments=gripper_controllers))

    return actions

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
