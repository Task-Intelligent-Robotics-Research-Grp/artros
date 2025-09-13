from launch                     import LaunchDescription
from launch.actions             import OpaqueFunction
from launch.substitutions       import (LaunchConfiguration,
                                        PathJoinSubstitution)
from launch_ros.substitutions   import FindPackageShare
from aist_bringup.launch_common import declare_launch_arguments, load_config


launch_arguments = [
    {
        'name':        'config',
        'default':     'aist',
        'description': 'Name of the hardware configuration'
    },
    {
        'name':        'name',
        'default':     'precision_tool',
        'description': 'Name of the precision gripper'
    },
]


def launch_setup(context):
    config         = load_config(context)
    gripper_config = config['grippers'][LaunchConfiguration('name') \
                                       .perform(context)]

    return [
        IncludeLaunchDescriptionFile(
            PathJoinSubstitution([FindPackageShare('aist_precision_gripper'),
                                  'launch', 'launch.py']),
            launch_arguments=[('name',     LaunchConfiguration('name')),
                              ('usb_port', gripper_config['usb_port']),
                              ('motor_id', gripper_config['motor_id'])])]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
