from launch                     import LaunchDescription
from launch.actions             import IncludeLaunchDescription, OpaqueFunction
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
        'default':     'a_bot_grippers',
        'description': 'Name of the set of Robotiq grippers'
    },
]


def launch_setup(context):
    config         = load_config(context)
    devices_config = config['grippers'][LaunchConfiguration('name')
                                         .perform(context)]
    gripper_names  = [gripper_name
                      for gripper_name in devices_config.get('grippers', {})]
    gripper_types  = [gripper_props['type']
                      for gripper_props in devices_config.get('grippers', {})
                      .values()]
    return [
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('aist_robotiq'), 'launch',
                                  'launch.py']),
            launch_arguments=[
                ('param_file',    PathJoinSubstitution([
                                      FindPackageShare('aist_bringup'),
                                      'config', 'devices', 'devices.yaml'])),
                ('gripper_names', ','.join(gripper_names)),
                ('gripper_types', ','.join(gripper_types)),
                ('container',     [LaunchConfiguration('name'), '_container']),
                ('driver_ns',     [LaunchConfiguration('name'), '_driver'])
            ])
    ]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
