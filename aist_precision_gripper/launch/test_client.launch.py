from launch               import LaunchDescription
from launch.actions       import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions   import Node

launch_arguments = [
    {'name':        'controller_ns',
     'default':     'precision_gripper_controller',
     'description': 'namespace of the controller'}]

def declare_launch_arguments(args):
    return [DeclareLaunchArgument(arg['name'],
                                  default_value=arg['default'],
                                  description=arg['description']) \
            for arg in args]

def launch_setup(context):
    return [Node(name='test_client',
                 package='aist_precision_gripper',
                 executable='test_client.py',
                 parameters=[
                     {'controller_ns': LaunchConfiguration('controller_ns')}],
                 prefix=['xterm -fn 10x20 -e'],
                 output='screen')]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
