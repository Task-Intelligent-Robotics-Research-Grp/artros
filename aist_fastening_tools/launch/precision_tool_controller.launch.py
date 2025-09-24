from launch                     import LaunchDescription
from launch.actions             import (IncludeLaunchDescription,
                                        OpaqueFunction)
from launch.substitutions       import (LaunchConfiguration, ThisLaunchFileDir,
                                        PathJoinSubstitution)
from aist_bringup.launch_common import declare_launch_arguments

launch_arguments = [
    {
        'name':        'tool_name',
        'default':     'precision_tool',
        'description': 'tool name'
    },
    {
        'name':        'tool_type',
        'default':     'PrecisionTool',
        'description': 'tool type'
    },
    {
        'name':        'motor_id',
        'default':     '1',
        'description': 'ID of the Dynamixel motor'
    },
    {
        'name':        'usb_port',
        'default':     '/dev/ttyUSB1',
        'description': 'device name of the USB port'
    },
    {
        'name':        'baud_rate',
        'default':     '57600',
        'description': 'baud rate of the serial communication'
    },
    {
        'name':        'container',
        'default':     'screw_tools_container',
        'description': 'name of the component container'
    },
    {
        'name':        'driver_ns',
        'default':     'screw_tools_driver',
        'description': 'name of the Dynamixel driver'
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

def launch_setup(context):
    return [
        IncludeLaunchDescription(
            PathJoinSubstitution([ThisLaunchFileDir(),
                                  'dynamixel_controllers.launch.py']),
            launch_arguments=[
                ('tool_names', LaunchConfiguration('tool_name')),
                ('tool_types', LaunchConfiguration('tool_type')),
                ('motor_ids',  LaunchConfiguration('motor_id')),
                ('usb_port',   LaunchConfiguration('usb_port')),
                ('baud_rate',  LaunchConfiguration('baud_rate')),
                ('container',  [LaunchConfiguration('tool_name'),
                                '_container']),
                ('driver_ns',  [LaunchConfiguration('tool_name'),
                                '_driver'])])]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
