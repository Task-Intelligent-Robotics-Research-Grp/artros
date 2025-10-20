from launch                     import LaunchDescription
from launch.actions             import IncludeLaunchDescription, OpaqueFunction
from launch.substitutions       import (LaunchConfiguration,
                                        PathJoinSubstitution)
from launch_ros.substitutions   import FindPackageShare
from aist_bringup.launch_common import declare_launch_arguments, load_config
from aist_utility.fileio        import filepath_from_url


launch_arguments = [
    {
        'name':        'config',
        'default':     'aist',
        'description': 'Name of the hardware configuration'
    },
    {
        'name':        'name',
        'default':     'suction_tools',
        'description': 'Name of the Dynamixel device group'
    },
]

def launch_setup(context):
    config       = load_config(context)
    tools_config = config['grippers'][LaunchConfiguration('name') \
                                      .perform(context)]
    tool_names = [tool_name for tool_name in tools_config.get('grippers', {})]

    return [
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('aist_fastening_tools'),
                                  'launch', 'ur_io_devices.launch.py']),
            launch_arguments=[
                ('param_file', filepath_from_url(tools_config['param_file'])),
                ('tool_names', ','.join(tool_names)),
                ('container',  [LaunchConfiguration('name'), '_container']),
                ('driver_ns',  tools_config['driver_ns'])])]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
