from launch                     import LaunchDescription
from launch.actions             import OpaqueFunction, IncludeLaunchDescription
from launch.substitutions       import (LaunchConfiguration, ThisLaunchFileDir,
                                        PathJoinSubstitution)
from launch_ros.actions         import Node
from aist_bringup.launch_common import declare_launch_arguments

launch_arguments = [
    {
        'name':        'tool_name',
        'default':     'suction_tool',
        'description': 'name of the tool'
    }
]

def launch_setup(context):
    return [
        IncludeLaunchDescription(
            PathJoinSubstitution([ThisLaunchFileDir(),
                                  'suction_tools.launch.py']),
            launch_arguments=[
                ('tool_names', LaunchConfiguration('tool_name')),
                ('tool_types', 'SuctionTool'),
                ('container',  'suction_tools_container'),
                ('driver_ns',  'screw_tools_driver'),
            ]),
        Node(name='suction_tool_test',
             package='aist_fastening_tools',
             executable='suction_tool_test.py',
             parameters=[{'tool_name': LaunchConfiguration('tool_name')}],
             prefix=['xterm -fn 7x14 -e'],
             output='screen')
    ]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
