from launch                            import LaunchDescription
from launch.actions                    import (DeclareLaunchArgument,
                                               OpaqueFunction)
from launch.substitutions              import (LaunchConfiguration,
                                               PathJoinSubstitution,
                                               IfElseSubstitution)
from launch_ros.actions                import Node
from launch_ros.substitutions          import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile

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
    {
        'name':        'driver',
        'default':     'urcap',
        'description': 'driver type'
        'choices':     ['urcap', 'tcp', 'rtu']
    },
    {
        'name':        'ip_or_dev',
        'default':     '10.66.171.40',
        'description': 'IP address or device file'
    },
    {
        'name':        'slave_id',
        'default':     '9',
        'description': 'slave ID'
    },
    {
        'name':        'log_level',
        'default':     'info',
        'description': 'debug log level',
        'choices':     ['debug', 'info', 'warn', 'error', 'fatal']
    }
]

def declare_launch_arguments(args):
    return [DeclareLaunchArgument(arg['name'],
                                  default_value=arg.get('default'),
                                  description=arg.get('description'),
                                  choices=arg.get('choices')) \
            for arg in args]

def launch_setup(context):
    prefix     = LaunchConfiguration('prefix')
    param_file = ParameterFile(PathJoinSubstitution(
                                 [FindPackageShare('aist_robotiq'), 'config',
                                  [LaunchConfiguration('device'), '.yaml']]),
                               allow_substs=True)
    controller = IfElseSubstitution(
                     EqualsSubstitution(
                         LaunchConfiguration('device'), 'robotiq_epick'),
                         'epick_controller.py', 'cmodel_controller.py')
    return [Node(name=[prefix, 'driver'],
                 package='aist_robotiq',
                 executable=['cmodel_', LaunchConfiguration('driver'),
                             '_driver.py'],
                 remappings=[('/status',  [prefix, 'controller/status']),
                             ('/command', [prefix, 'controller/command'])],
                 output='screen',
                 arguments=[LaunchConfiguration('ip_or_dev'),
                            LaunchConfiguration('slave_id')]),
            Node(name=[prefix, 'controller'],
                 package='aist_robotiq',
                 executable=controller,
                 parameters=[param_file],
                 output='screen',
                 arguments=['--ros-args', '--log-level',
                            LaunchConfiguration('log_level')])]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
