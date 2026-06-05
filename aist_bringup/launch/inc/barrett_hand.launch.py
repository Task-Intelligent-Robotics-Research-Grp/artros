from launch                     import LaunchDescription
from launch.actions             import (IncludeLaunchDescription,
                                        OpaqueFunction)
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
        'default':     'barrett_hand',
        'description': 'Name of the hand'
    },
]

def launch_setup(context):
    config       = load_config(context)
    gripper_name = LaunchConfiguration('name').perform(context)
    gripper_type = config['grippers'][gripper_name]['type']
    return [
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('aist_barrett'), 'launch',
                'launch.py']),
            launch_arguments=[
                ('param_file',   PathJoinSubstitution([
                                     FindPackageShare('aist_bringup'),
                                     'config', 'devices', 'devices.yaml'])),
                ('gripper_name', gripper_name),
                ('container',    [LaunchConfiguration('name'), '_container']),
            ])
    ]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
