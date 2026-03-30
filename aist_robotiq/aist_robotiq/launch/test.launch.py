from launch                     import LaunchDescription
from launch.actions             import OpaqueFunction, IncludeLaunchDescription
from launch.substitutions       import (LaunchConfiguration, ThisLaunchFileDir,
                                        PathJoinSubstitution,
                                        IfElseSubstitution, EqualsSubstitution)
from launch_ros.actions         import Node
from aist_bringup.launch_common import declare_launch_arguments

launch_arguments = [
    {
        'name':        'device_name',
        'default':     'robotiq_85',
        'description': 'name of the device',
        'choices':     ['robotiq_85', 'robotiq_140', 'robotiq_hande',
                        'robotiq_3f', 'robotiq_epick'],
    },
]

def launch_setup(context):
    device_type = IfElseSubstitution(
                      EqualsSubstitution(
                          LaunchConfiguration('device_name'), 'robotiq_epick'),
                      'RobotiqEPick', 'RobotiqGripper')
    client_type = IfElseSubstitution(
                      EqualsSubstitution(device_type, 'RobotiqEPick'),
                      'epick', 'cmodel')
    return [
        IncludeLaunchDescription(
            PathJoinSubstitution([ThisLaunchFileDir(), 'launch.py']),
            launch_arguments=[
                ('device_names', LaunchConfiguration('device_name')),
                ('device_types', device_type),
                ('driver_ns',    [LaunchConfiguration('device_name'),
                                  '_driver']),
            ]),
        Node(name=['test_', client_type, '_client'],
             package='aist_robotiq',
             executable=['test_', client_type, '_client.py'],
             parameters=[{'device_name': LaunchConfiguration('device_name')}],
             prefix=['xterm -fn 7x14 -e'],
             output='screen'),
    ]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
