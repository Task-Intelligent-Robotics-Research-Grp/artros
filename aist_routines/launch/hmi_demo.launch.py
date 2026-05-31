from launch                     import LaunchDescription
from launch.actions             import (IncludeLaunchDescription,
                                        OpaqueFunction, RegisterEventHandler)
from launch.substitutions       import (LaunchConfiguration, ThisLaunchFileDir,
                                        PathJoinSubstitution)
from launch.event_handlers      import OnProcessStart
from launch_ros.actions         import Node
from launch_ros.substitutions   import FindPackageShare
from aist_bringup.launch_common import declare_launch_arguments


launch_arguments = [
    {
        'name':        'log_level',
        'default':     'info',
        'description': 'debug log level',
        'choices':     ['debug', 'info', 'warn', 'error', 'fatal']
    },
    {
        'name':        'output',
        'default':     'screen',
        'description': 'pipe node output',
        'choices':     ['screen', 'log', 'both']
    },
]

def launch_setup(context):
    hdc_node = Node(name='hmi_demo_container',
                    package='rclcpp_components',
                    executable='component_container_mt',
                    output=LaunchConfiguration('output'),
                    arguments=['--ros-args', '--log-level',
                               LaunchConfiguration('log_level')])
    return [
        IncludeLaunchDescription(
            PathJoinSubstitution([ThisLaunchFileDir(), 'kitting.launch.py'])),
        hdc_node,
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare('aist_visualization'),
                 'launch', 'launch.py']),
            launch_arguments=[
                ('external_container', 'true'),
                ('container',          'hmi_demo_container'),
                ('param_file',         LaunchConfiguration('settings_file')),
            ]),
        RegisterEventHandler(
            OnProcessStart(
                target_action=hdc_node,
                on_start=[
                    IncludeLaunchDescription(
                        PathJoinSubstitution(
                            [FindPackageShare('nep_bridge'),
                             'launch', 'launch.py']),
                        launch_arguments=[
                            ('param_file',
                             LaunchConfiguration('settings_file')),
                        ]),
                ])),
    ]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
