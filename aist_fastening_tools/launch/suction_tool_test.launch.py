from launch                     import LaunchDescription
from launch.actions             import OpaqueFunction
from launch.substitutions       import LaunchConfiguration
from launch_ros.actions         import Node
from aist_bringup.launch_common import declare_launch_arguments

launch_arguments = [
    {
        'name':        'controller_ns',
        'default':     'screw_tool_m4_controller',
        'description': 'namespace of the gripper controller'
    }
]

def launch_setup(context):
    return [Node(name='suction_tool_test',
                 package='aist_fastening_tools',
                 executable='suction_tool_test.py',
                 parameters=[
                     {'controller_ns': LaunchConfiguration('controller_ns')}],
                 prefix=['xterm -fn 7x14 -e'],
                 output='screen')]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
