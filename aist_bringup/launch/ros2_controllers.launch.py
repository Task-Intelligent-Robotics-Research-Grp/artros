from launch                            import LaunchDescription
from launch.actions                    import (SetLaunchConfiguration,
                                               IncludeLaunchDescription,
                                               OpaqueFunction, GroupAction,
                                               RegisterEventHandler)
from launch.conditions                 import IfCondition, UnlessCondition
from launch.event_handlers             import OnProcessExit
from launch.substitutions              import (Command, FindExecutable,
                                               LaunchConfiguration,
                                               PathJoinSubstitution)
from launch_ros.actions                import Node, PushROSNamespace
from launch_ros.substitutions          import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile
from aist_bringup.launch_common        import (declare_launch_arguments,
                                               load_config, get_device_props,
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

def create_node(ns, config, context):
    sim   = LaunchConfiguration('sim').perform(context) in ('true', 'True')
    name  = ns.split('/')[-1]
    props = get_device_props(config['type'])

    active_controllers   = config.get('active_controllers', []) \
                         + config.get('consistent_controllers', [])
    inactive_controllers = config.get('inactive_controllers', [])
    if sim:
        SetLaunchConfiguration('name', name).execute(context)
        SetLaunchConfiguration('update_rate',
                               str(props['update_rate'])).execute(context)
        SetLaunchConfiguration('speed_scaling_interface_name',
                               '""').execute(context)
        instantiate_file(context, props['controllers_template'],
                         '/tmp/' + name + '_controllers.yaml')
    else:
        active_controllers += config.get('real_consistent_controllers', [])
        inactive_controllers += config.get('real_inactive_controllers', [])

    actions = [
        PushROSNamespace(name),
        SetLaunchConfiguration('name', name),
        SetLaunchConfiguration('update_rate', str(props['update_rate'])),
        SetLaunchConfiguration('speed_scaling_interface_name',
                               '' if sim else
                               name + '_speed_scaling/speed_scaling_factor'),
        Node(package='aist_bringup',
             executable='append_ros2_control',
             parameters=[
                 {'ros2_control_descriptions':
                  ParameterValue(
                      Command([
                          FindExecutable(name='xacro'), ' ',
                          props['ros2_control_file'],
                          ' name:=', LaunchConfiguration('name'),
                          ' sim:=',  LaunchConfiguration('sim'),
                      ]),
                      value_type=str)
                 }
             ],
             remappings=[
                 ('robot_description_in', '/robot_description')
             ],
             output='screen'),
        Node(package='controller_manager',
             executable='ros2_control_node',
             parameters=[
                 ParameterFile(props['controllers_template'],
                               allow_substs=True),
             ],
             output='screen',
             condition=UnlessCondition(LaunchConfiguration('sim'))),
        Node(package='controller_manager',
             executable='spawner',
             arguments=['--controller-manager-timeout', '15'] + active_controllers,
             output='screen'),
    ] + ([
        Node(package='controller_manager',
             executable='spawner',
             arguments=['--controller-manager-timeout', '15', '--inactive'] + inactive_controllers,
             output='screen')
    ] if len(inactive_controllers) > 0 else [])
    print('### name=%s, active_controllers=%s, inactive_controllers=%s'
          % (name, active_controllers, inactive_controllers))
    return GroupAction(actions=actions)

def create_actions(ns, config, context):
    # 'ns' represents a node.
    if 'type' in config:
        return create_node(ns, config, context), [ns]

    # 'ns' represents a namespace.
    actions = [PushROSNamespace(ns.split('/')[-1])] if ns != '' else []
    namespaces = []
    for n, c in config.items():
        action, ns_list = create_actions(ns + '/' + n, c, context)
        actions.append(action)
        namespaces += ns_list
    return GroupAction(actions=actions), namespaces

def launch_setup(context):
    action, namespaces = create_actions('', load_config(context), context)

    awaited_actions = [
        Node(package='joint_state_publisher',
             executable='joint_state_publisher',
             parameters=[
                 {'rate':         50, #LaunchConfiguration('update_rate'),
                  'use_sim_time': LaunchConfiguration('sim'),
                  'source_list':  [namespace + '/joint_states' \
                                   for namespace in namespaces]}
             ],
             output='screen'),
        GroupAction(
            actions=[
                Node(package='ros_gz_sim',
                     executable='create',
                     arguments=[
                         '-name',  LaunchConfiguration('config'),
                         '-topic', 'robot_description'
                     ],
                     output='screen'),
                IncludeLaunchDescription(
                    PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                          'launch', 'gz_sim.launch.py']),
                    launch_arguments=[
                        ('gz_args',
                         [' -r -v 4 empty.sdf', ' --physics-engine',
                          ' gz-physics-bullet-featherstone-plugin'])
                    ]),
                ],
            condition=IfCondition(LaunchConfiguration('sim'))),
    ]
    awaited_actions.append(action)

    robot_description \
        = ParameterValue(Command([FindExecutable(name='xacro'),
                                  ' ',
                                  PathJoinSubstitution(
                                      [FindPackageShare('aist_description'),
                                       'scenes', 'urdf',
                                       [LaunchConfiguration('config'),
                                          '_base_scene.urdf.xacro']]),
                                  ' scene:=', LaunchConfiguration('scene'),
                                  ' sim:=',   LaunchConfiguration('sim')]),
                         value_type=str)
    wait_for_robot_description \
        = Node(package='aist_bringup',
               executable='wait_for_robot_description',
               output='screen')
    return [
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             parameters=[
                 {'use_sim_time':      LaunchConfiguration('sim'),
                  'robot_description': robot_description}
             ],
             output='screen'),
        wait_for_robot_description,
        RegisterEventHandler(
            OnProcessExit(target_action=wait_for_robot_description,
                          on_exit=awaited_actions)),
    ]


def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
