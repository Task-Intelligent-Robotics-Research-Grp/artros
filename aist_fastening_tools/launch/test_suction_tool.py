from launch                     import LaunchDescription
from launch.actions             import OpaqueFunction, IncludeLaunchDescription
from launch.substitutions       import (LaunchConfiguration, ThisLaunchFileDir,
                                        PathJoinSubstitution)
from launch_ros.actions         import Node
from aist_bringup.launch_common import declare_launch_arguments

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
        Node(name='test_suction_tool',
             package='aist_fastening_tools',
             executable='test_suction_tool.py',
             parameters=[{'device_name': LaunchConfiguration('device_name')}],
             prefix=['gnome-terminal --geometry=80x60 --'],
             output='screen')
    ]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
