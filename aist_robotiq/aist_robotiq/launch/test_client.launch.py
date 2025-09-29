from launch               import LaunchDescription
from launch.actions       import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (LaunchConfiguration, IfElseSubstitution,
                                  EqualsSubstitution)
from launch_ros.actions   import Node

launch_arguments = [
    {
        'name':        'prefix',
        'default':     'a_bot_gripper_',
        'description': 'prefix of controller'
    },
    {
        'name':        'device',
        'default':     'robotiq_140',
        'description': 'device type',
        'choices':     ['robotiq_85', 'robotiq_140', 'robotiq_hande',
                        'robotiq_epick']
    },
]

def declare_launch_arguments(args):
    return [DeclareLaunchArgument(arg['name'],
                                  default_value=arg.get('default'),
                                  description=arg.get('description'),
                                  choices=arg.get('choices')) \
            for arg in args]

def launch_setup(context):
    client_type = IfElseSubstitution(
                      EqualsSubstitution(
                          LaunchConfiguration('device'), 'epick'),
                      'epick', 'cmodel')

    return [Node(name=['test_', client_type, '_client'],
                 package='aist_robotiq',
                 executable=['test_', client_type, '_client.py'],
                 parameters=[{'prefix': LaunchConfiguration('prefix')}],
                 prefix=['xterm -fn 7x14 -e'],
                 output='screen')]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
