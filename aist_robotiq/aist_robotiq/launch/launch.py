import yaml
from launch                   import LaunchDescription
from launch.actions           import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions     import (LaunchConfiguration,
                                      PathJoinSubstitution, ThisLaunchFileDir)
from launch.conditions        import IfCondition, UnlessCondition
from launch_ros.actions       import Node

launch_arguments = [
    {'name':        'prefix',
     'default':     'a_bot_gripper_',
     'description': 'prefix of controller'},
    {'name':        'device',
     'default':     'robotiq_140',
     'description': 'device type[robotiq_85|robotiq_140|robotiq_hande|robotiq_epick]'},
    {'name':        'driver',
     'default':     'urcap',
     'description': 'driver type[urcap|tcp|rtu]'},
    {'name':        'ip_or_dev',
     'default':     '10.66.171.40',
     'description': 'IP address or device file'},
    {'name':        'slave_id',
     'default':     '9',
     'description': 'slave ID'},
    {'name':        'log_level',
     'default':     'info',
     'description': 'debug log level [DEBUG|INFO|WARN|ERROR|FATAL]'}]

def declare_launch_arguments(args, defaults={}):
    num_to_str = lambda x : str(x) if isinstance(x, (bool, int, float)) else x
    return [DeclareLaunchArgument(
                arg['name'],
                default_value=num_to_str(defaults.get(arg['name'],
                                                      arg['default'])),
                description=arg['description']) \
            for arg in args]

def load_parameters(config_file):
    if config_file == '':
        return {}
    with open(config_file, 'r') as f:
        return yaml.load(f, Loader=yaml.SafeLoader)

def launch_setup(context):
    prefix = LaunchConfiguration('prefix').perform(context)
    device = LaunchConfiguration('device').perform(context)
    driver = LaunchConfiguration('driver').perform(context)
    params = load_parameters(PathJoinSubstitution(
                                 [ThisLaunchFileDir(), '..',
                                  'config', device + '.yaml']).perform(context))
    if device == 'robotiq_epick':
        controller_type = 'epick'
    else:
        controller_type = 'cmodel'
        params |= {'joint_name': prefix + 'joint_finger'}
    return [Node(name=prefix + 'driver',
                 package='aist_robotiq',
                 executable='cmodel_' + driver + '_driver.py',
                 remappings=[('/status',  prefix + 'controller/status'),
                             ('/command', prefix + 'controller/command')],
                 output='screen',
                 arguments=[LaunchConfiguration('ip_or_dev'),
                            LaunchConfiguration('slave_id')]),
            Node(name=prefix + 'controller',
                 package='aist_robotiq',
                 executable=controller_type + '_controller.py',
                 parameters=[params],
                 output='screen',
                 arguments=['--ros-args', '--log-level',
                            LaunchConfiguration('log_level')])]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
