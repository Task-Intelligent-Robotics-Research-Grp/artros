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
from typing                            import List

launch_arguments = [
    {
        'name':        'config',
        'default':     'aist_new',
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

def create_node(name, config, context):
    sim = LaunchConfiguration('sim').perform(context) in ('true', 'True')
    type  = config['type']
    props = get_device_props(type)

    actions              = []
    rc2_desc             = None
    param_file           = None
    active_controllers   = []
    inactive_controllers = []
    if type == 'UR' or type == 'URe' or type == 'LBR':
        actions = [
            SetLaunchConfiguration('arm_name', name),
            SetLaunchConfiguration('update_rate',  str(props['update_rate'])),
            SetLaunchConfiguration('speed_scaling_interface_name',
                                   name +
                                   '_speed_scaling/speed_scaling_factor'),
        ]
        rc2_desc = ParameterValue(
                       Command([FindExecutable(name='xacro'), ' ',
                                arm_props['ros2_control_file'],
                                ' name:=', LaunchConfiguration('arm_name'),
                                ' sim:=',  LaunchConfiguration('sim')]),
                       value_type=str)
        active_controllers   = config['active_controllers'] \
                             + config.get('consistent_controllers', [])
        inactive_controllers = config.get('inactive_controllers', [])
        if sim:
            SetLaunchConfiguration('arm_name', name).execute(context)
            SetLaunchConfiguration('update_rate',
                                   str(props['update_rate'])).execute(context)
            SetLaunchConfiguration('speed_scaling_interface_name',
                                   '""').execute(context)
            instantiate_file(context, props['controllers_template'],
                             '/tmp/' + name + '_controllers.yaml')
        else:
            param_file = props['controllers_template']
            active_controllers += config.get('real_consistent_controllers', [])
            inactive_controllers += config.get('real_inactive_controllers', [])
    elif type == 'RobotiqGripper':
        if sim:
            SetLaunchConfiguration('gripper_name', name).execute(context)
            instantiate_file(context, props['gz_controllers_template'],
                             '/tmp/' + name + '_controllers.yaml')
            rc2_desc = props.get('ros2_control_file')
            active_controllers = [name + '_controller']

    print('### name=%s, active_controllers=%s, inactive_controllers=%s'
          % (name, active_controllers, inactive_controllers))
    return actions, rc2_desc, param_file, active_controllers, inactive_controllers, []

def create_actions(ns, config, context):
    print('### OK: %s' % ns)
    name = ns.split('/')[-1]

    # 'ns' represents a node.
    if 'type' in config:
        return create_node(name, config, context)

    # 'ns' represents a namespace.
    actions              = []
    rc2_descs            = []
    param_files          = []
    namespaces           = []
    active_controllers   = []
    inactive_controllers = []
    if name != '':
        actions.append(PushROSNamespace(name))
        namespaces.append(ns)

    for n, c in config.items():
        acts, rc2_desc, param_file, active_list, inactive_list, ns_list = create_actions(ns + '/' + n, c, context)
        actions += acts
        if rc2_desc is not None:
            rc2_descs.append(rc2_desc)
        if param_file is not None:
            param_files.append(param_file)
        active_controllers   += active_list
        inactive_controllers += inactive_list
        namespaces           += ns_list

    if rc2_descs:
        print('### rc2_descs=%s' % rc2_descs)
        actions.append(
            Node(package='aist_bringup',
                 executable='append_ros2_control',
                 parameters=[
                     {'ros2_control_descriptions':
                      ParameterValue(rc2_descs, value_type=str)}
                 ],
                 remappings=[
                     ('robot_description_in', '/robot_description')
                 ],
                 output='screen'))
    if param_files:
        actions.append(
            Node(package='controller_manager',
                 executable='ros2_control_node',
                 parameters=param_files,
                 output='screen',
                 condition=UnlessCondition(LaunchConfiguration('sim'))))
    if active_controllers:
        actions.append(
            Node(name='active_controllers_spawner',
                 package='controller_manager',
                 executable='spawner',
                 arguments=active_controllers,
                 output='screen'))
    if inactive_controllers:
        actions.append(
            Node(name='inactive_controllers_spawner',
                 package='controller_manager',
                 executable='spawner',
                 arguments=['--inactive'] + inactive_controllers,
                 output='screen'))


    if name != '':
        return [GroupAction(actions=actions)], None, None, [], [], namespaces
    else:
        return actions, None, None, [], [], namespaces

def launch_setup(context):
    actions, _, _, _, _, namespaces = create_actions('', load_config(context),
                                                     context)

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
    ] + actions


    robot_description = ParameterValue(
                            Command([FindExecutable(name='xacro'),
                                     ' ',
                                     PathJoinSubstitution(
                                         [FindPackageShare('aist_description'),
                                          'scenes', 'urdf',
#                                          [LaunchConfiguration('config'),
#                                           '_base_scene.urdf.xacro']]),
                                          'aist_base_scene.urdf.xacro']),
                                     ' scene:=', LaunchConfiguration('scene'),
                                     ' sim:=',   LaunchConfiguration('sim')]),
                            value_type=str)
    wait_for_robot_description = Node(package='aist_bringup',
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
