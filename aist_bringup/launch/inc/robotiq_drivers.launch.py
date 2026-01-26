from launch                     import LaunchDescription
from launch.actions             import IncludeLaunchDescription, OpaqueFunction
from launch.substitutions       import (LaunchConfiguration,
                                        PathJoinSubstitution)
from launch_ros.substitutions   import FindPackageShare
from aist_bringup.launch_common import (declare_launch_arguments,
                                        load_gripper_config)


launch_arguments = [
    {
        'name':        'name',
        'default':     'a_bot_gripper',
        'description': 'Name of the Robotiq gripper'
    },
]


def launch_setup(context):
    gripper_name   = LaunchConfiguration('name').perform(context)
    gripper_config = load_gripper_config(gripper_name)
    return [
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('aist_robotiq'), 'launch',
                                  'launch.py']),
            launch_arguments=[('prefix', [LaunchConfiguration('name'), '_']),
                              ('device',    gripper_config['device']),
                              ('driver',    gripper_config['driver']),
                              ('ip_or_dev', gripper_config['ip_or_dev']),
                              ('slave_id',  str(gripper_config['slave_id']))])]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
