from launch                     import LaunchDescription
from launch.actions             import OpaqueFunction
from launch.substitutions       import LaunchConfiguration
from launch_ros.actions         import Node
from aist_bringup.launch_common import declare_launch_arguments

launch_arguments = [
    {
        'name':        'controller_ns',
        'default':     'precision_tool_controller',
        'description': 'namespace of the gripper controller'
    }
]

def launch_setup(context):
    return [Node(name='test_client',
                 package='aist_fastening_tools',
                 executable='gripper_test.py',
                 parameters=[
                     {'controller_ns': LaunchConfiguration('controller_ns')}],
                 prefix=['xterm -fn 7x14 -e'],
                 output='screen')]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
