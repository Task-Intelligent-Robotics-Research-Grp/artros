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
        'name':        'gripper_name',
        'default':     'a_bot_gripper',
        'description': 'Name of the Robotiq gripper'
    },
]


def launch_setup(context):
    config         = load_config(context)
    gripper_config = config['grippers'][LaunchConfiguration('gripper_name') \
                                       .perform(context)]
    device         = gripper_config['device']

    return [
        IncludeLaunchDescriptionFile(
            PathJoinSubstitution([FindPackageShare('aist_robotiq'), 'launch',
                                  'launch.py']),
            launch_arguments=[('prefix', [LaunchConfiguration('gripper_name'),
                                          '_']),
                              ('device',    gripper_config['device']),
                              ('driver',    gripper_config['driver']),
                              ('ip_or_dev', gripper_config['ip_or_dev']),
                              ('slave_id',  gripper_config['slave_id'])])]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
