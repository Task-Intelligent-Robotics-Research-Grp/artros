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

def create_node(name, config, sim):
    type  = config['type']
    props = get_device_props(type)
    param_file = props.get('controllers_template')
    if param_file is None and sim:
        param_file = props.get('gz_controllers_template')
    rc2_desc = props.get('ros2_control_file')
    return [], rc2_desc, param_file

def create_actions(name, config, sim):
    # 'name' represents a node.
    if 'type' in config:
        return create_node(name, config, sim)

    # 'name' represents a namespace.
    actions = []
    if name != '':
        actions.append(PushROSNamespace(name))

    rc2_descs   = []
    param_files = []
    for n, c in config.items():
        acts, rc2_desc, param_file = create_actions(n, c, sim)
        actions += acts
        if rc2_desc is not None:
            rc2_descs.append(rc2_desc)
        if param_file is not None:
            param_files.append(param_file)
    if rc2_descs:
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

    if name != '':
        return [GroupAction(actions=actions)], None, None
    else:
        return actions, None, None

def launch_setup(context):
    return create_actions('', load_config(context))

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
