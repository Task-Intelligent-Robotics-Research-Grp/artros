from launch                     import LaunchDescription
from launch.actions             import OpaqueFunction, IncludeLaunchDescription
from launch.substitutions       import (LaunchConfiguration, ThisLaunchFileDir,
                                        PathJoinSubstitution)
from launch_ros.actions         import Node
from aist_bringup.launch_common import declare_launch_arguments

launch_arguments = [
    {
        'name':        'device_name',
        'default':     'precision_tool',
        'description': 'device name of the tool'
    }
]

def launch_setup(context):
    return [
        IncludeLaunchDescription(
            PathJoinSubstitution([ThisLaunchFileDir(),
                                  'dynamixel_devices.launch.py']),
            launch_arguments=[
                ('device_names', LaunchConfiguration('device_name')),
                ('device_types', 'PrecisionTool'),
                ('container',    'precision_tools_container'),
                ('driver_ns',    'precision_tools_driver'),
            ]),
        Node(name='precision_tool_test',
             package='aist_fastening_tools',
             executable='precision_tool_test.py',
             parameters=[
                 {'device_name': LaunchConfiguration('device_name')},
             ],
             prefix=['xterm -fn 7x14 -e'],
             output='screen')
    ]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
