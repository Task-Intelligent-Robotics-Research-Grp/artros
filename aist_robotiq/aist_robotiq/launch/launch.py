from launch                            import LaunchDescription
from launch.actions                    import (DeclareLaunchArgument,
                                               OpaqueFunction)
from launch.substitutions              import (LaunchConfiguration,
                                               PathJoinSubstitution,
                                               IfElseSubstitution,
                                               EqualsSubstitution)
from launch_ros.actions                import Node, LoadComposableNodes
from launch_ros.descriptions           import ComposableNode
from launch_ros.substitutions          import FindPackageShare


launch_arguments = [
    {
        'name':        'param_file',
        'default':     PathJoinSubstitution([FindPackageShare('aist_robotiq'),
                                             'config', 'default.yaml']),
        'description': 'absolute path to configuration file'
    },
    {
        'name':        'gripper_names',
        'default':     'robotiq_85',
        'description': 'comma-separated list of device names'
    },
    {
        'name':        'gripper_types',
        'default':     'RobotiqGripper',        # RobotiqGripper/RobotiqEPick
        'description': 'comma-separated list of device types'
    },
    {
        'name':        'driver_ns',
        'default':     'robotiq_85_driver',
        'description': 'name of the driver for Robotiq devices'
    },
    {
        'name':        'driver_type',
        'default':     'urcap',
        'description': 'driver type',
        'choices':     ['urcap', 'tcp', 'rtu']
    },
   {
        'name':        'container',
        'default':     'robotiq_grippers_container',
        'description': 'name of the component container'
    },
     {
        'name':        'log_level',
        'default':     'info',
        'description': 'debug log level',
        'choices':     ['debug', 'info', 'warn', 'error', 'fatal']
    },
    {
        'name':        'output',
        'default':     'both',
        'description': 'pipe node output',
        'choices':     ['screen', 'log', 'both']
    }
]

PLUGINS = {
    'RobotiqGripper': 'aist_robotiq::GripperController',
    'RobotiqSuction': 'aist_robotiq::SuctionController',
}

def declare_launch_arguments(args):
    return [DeclareLaunchArgument(arg['name'],
                                  default_value=arg.get('default'),
                                  description=arg.get('description'),
                                  choices=arg.get('choices')) \
            for arg in args]

def launch_setup(context):

    composable_nodes = []
    for gripper_name, device_type \
          in zip(LaunchConfiguration('gripper_names').perform(context)
                 .split(','),
                 LaunchConfiguration('gripper_types').perform(context)
                 .split(',')):
        composable_nodes.append(
            ComposableNode(name=gripper_name + '_controller',
                           package='aist_robotiq',
                           plugin=PLUGINS[device_type],
                           parameters=[
                               LaunchConfiguration('param_file'),
                           ],
                           remappings=[
                               ('/status',  [LaunchConfiguration('driver_ns'),
                                             '/status']),
                               ('/command', [LaunchConfiguration('driver_ns'),
                                             '/command'])
                           ],
                           extra_arguments=[
                               {'use_intra_process_comms': True}
                           ]))
    return [
        Node(name=LaunchConfiguration('driver_ns'),
             package='aist_robotiq',
             executable='cmodel_driver.py',
             parameters=[
                 LaunchConfiguration('param_file')
             ],
             arguments=[
                 LaunchConfiguration('driver_type')
             ],
             output=LaunchConfiguration('output')),
        Node(name=LaunchConfiguration('container'),
             package='rclcpp_components',
             executable='component_container_mt',
             output=LaunchConfiguration('output'),
             arguments=[
                 '--ros-args', '--log-level', LaunchConfiguration('log_level')
             ]),
        LoadComposableNodes(target_container=LaunchConfiguration('container'),
                            composable_node_descriptions=composable_nodes),
    ]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
