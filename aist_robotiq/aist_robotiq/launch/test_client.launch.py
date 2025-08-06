from launch               import LaunchDescription
from launch.actions       import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions   import Node

launch_arguments = [
    {'name':        'prefix',
     'default':     'a_bot_gripper_',
     'description': 'prefix of controller'},
    {'name':        'device',
     'default':     'robotiq_140',
     'description': 'device type[robotiq_85|robotiq_140|robotiq_hande|robotiq_epick]'},
    {'name':        'sim',
     'default':     'false',
     'description': 'true if simulated with gazebo'}]

def declare_launch_arguments(args):
    return [DeclareLaunchArgument(arg['name'],
                                  default_value=arg['default'],
                                  description=arg['description']) \
            for arg in args]

def launch_setup(context):
    prefix = LaunchConfiguration('prefix').perform(context)
    device = LaunchConfiguration('device').perform(context)
    sim    = LaunchConfiguration('sim').perform(context)
    params = {'prefix': prefix}
    if device == 'robotiq_epick':
        client_type = 'epick'
    else:
        client_type = 'cmodel'

    return [Node(name='test_' + client_type + '_client',
                 package='aist_robotiq',
                 executable='test_' + client_type + '_client.py',
                 parameters=[params],
                 prefix=['xterm -fn 7x14 -e'],
                 output='screen')]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
