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
        'default':     'screw_tools',
        'description': 'Name of the Dynamixel device group'
    },
]

def launch_setup(context):
    config       = load_config(context)
    tools_config = config['grippers'][LaunchConfiguration('name')
                                      .perform(context)]
    tool_names   = [tool_name for tool_name in tools_config.get('tools', {})]
    tool_types   = [tool_props['type']
                    for tool_props in tools_config.get('tools', {}).values()]

    return [
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('aist_fastening_tools'), 'launch',
                'dynamixel_devices.launch.py']),
            launch_arguments=[
                ('param_file', PathJoinSubstitution([
                                   FindPackageShare('aist_bringup'), 'config',
                                   'devices', 'fastening_tools.yaml'])),
                ('tool_names', ','.join(tool_names)),
                ('tool_types', ','.join(tool_types)),
                ('container',  [LaunchConfiguration('name'), '_container']),
                ('driver_ns',  [LaunchConfiguration('name'), '_driver'])])]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
