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
        'default':     'suction_tools',
        'description': 'Name of the URio device group'
    },
]

def launch_setup(context):
    config         = load_config(context)
    devices_config = config['grippers'][LaunchConfiguration('name')
                                        .perform(context)]
    device_names   = [device_name
                      for device_name in devices_config.get('grippers', {})]
    return [
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('aist_fastening_tools'),
                                  'launch', 'ur_io_devices.launch.py']),
            launch_arguments=[
                ('param_file',   PathJoinSubstitution([
                                     FindPackageShare('aist_bringup'),
                                     'config', 'devices', 'grippers.yaml'])),
                ('device_names', ','.join(device_names)),
                ('container',    [LaunchConfiguration('name'), '_container'])
            ])
    ]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
