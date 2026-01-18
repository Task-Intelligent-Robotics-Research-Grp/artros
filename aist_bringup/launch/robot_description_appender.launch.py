from launch                            import LaunchDescription
from launch.actions                    import OpaqueFunction, GroupAction
from launch.substitutions              import (Command, FindExecutable,
                                               LaunchConfiguration)
from launch_ros.actions                import Node, PushROSNamespace
from launch_ros.parameter_descriptions import ParameterValue
from aist_bringup.launch_common        import (declare_launch_arguments,
                                               load_config, get_arm_props)

launch_arguments = [
    {
        'name':        'config',
        'default':     'aist',
        'description': 'Name of the hardware configuration'
    },
    {
        'name':        'arm_name',
        'default':     'a_bot',
        'description': 'Name of the arm'
    },
]

def launch_setup(context):
    arm_name  = LaunchConfiguration('arm_name').perform(context)
    config    = load_config(context)
    arm_props = get_arm_props(config['arms'][arm_name]['type'])
    return [
        PushROSNamespace(LaunchConfiguration('arm_name')),
        Node(package='aist_bringup',
             executable='robot_description_appender',
             parameters=[
                 {'extra_description':
                  ParameterValue(
                      Command([FindExecutable(name='xacro'),
                               ' ', arm_props['ros2_control_file'],
                               ' name:=', LaunchConfiguration('arm_name')]),
                      value_type=str)}
             ],
             remappings=[('robot_description_in', '/robot_description')],
             output='screen'),
    ]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
