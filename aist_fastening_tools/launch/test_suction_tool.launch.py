from launch                            import LaunchDescription
from launch.actions                    import (OpaqueFunction,
                                               IncludeLaunchDescription,
                                               GroupAction)
from launch.substitutions              import (LaunchConfiguration,
                                               ThisLaunchFileDir,
                                               PathJoinSubstitution,
                                               Command, FindExecutable)
from launch_ros.actions                import Node, PushROSNamespace
from launch_ros.substitutions          import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from aist_bringup.launch_common        import declare_launch_arguments

launch_arguments = [
    {
        'name':        'device_name',
        'default':     'suction_tool',
        'description': 'device name of the tool',
        'choices':     ['screw_tool_m3', 'screw_tool_m4', 'suction_tool',
                        'base_fixture'],
    }
]

def launch_setup(context):
    return [
        IncludeLaunchDescription(
            PathJoinSubstitution([ThisLaunchFileDir(),
                                  'ur_io_devices.launch.py']),
            launch_arguments=[
                ('device_names', LaunchConfiguration('device_name')),
            ]),
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             parameters=[
                 {'robot_description':
                  ParameterValue(
                      Command([
                          FindExecutable(name='xacro'), ' ',
                          PathJoinSubstitution([
                              FindPackageShare('ur_description'),
                              'urdf', 'ur.urdf.xacro'
                          ]),
                          ' name:=b_bot ur_type:=ur5e tf_prefix:=b_bot_'
                      ]),
                      value_type=str)}
             ],
             output='screen'),
        GroupAction(
            actions=[
                PushROSNamespace('b_bot'),
                Node(package='aist_bringup',
                     executable='append_ros2_control',
                     parameters=[
                         {'ros2_control_descriptions':
                          ParameterValue(
                              Command([
                                  FindExecutable(name='xacro'), ' ',
                                  PathJoinSubstitution([
                                      FindPackageShare('aist_bringup'),
                                      'urdf', 'ur.ros2_control.urdf.xacro'
                                  ]),
                                  ' name:=b_bot'
                              ]),
                              value_type=str)
                          },
                     ],
                     remappings=[
                         ('robot_description_in', '/robot_description')
                     ],
                     output='screen'),
                Node(package='controller_manager',
                     executable='ros2_control_node',
                     parameters=[
                         LaunchConfiguration('param_file'),
                     ],
                     output='screen'),
                Node(name='controllers_spawner',
                     package='controller_manager',
                     executable='spawner',
                     arguments=['io_and_status_controller'],
                     output='screen'),
            ]),
        Node(name='test_suction_tool',
             package='aist_fastening_tools',
             executable='test_suction_tool.py',

             parameters=[{'device_name': LaunchConfiguration('device_name')}],
             prefix=['gnome-terminal --tab --'],
             output='screen')
    ]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
