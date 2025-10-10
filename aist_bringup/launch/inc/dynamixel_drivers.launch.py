from launch                            import LaunchDescription
from launch.actions                    import (IncludeLaunchDescription,
                                               OpaqueFunction)
from launch.substitutions              import (LaunchConfiguration,
                                               PathJoinSubstitution)
from launch_ros.substitutions          import FindPackageShare
from aist_bringup.launch_common        import (declare_launch_arguments,
                                               load_config)


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
    tools_config = config['grippers'][LaunchConfiguration('name') \
                                      .perform(context)]
    tool_names = []
    tool_types = []
    motor_ids  = []
    for tool_name, tool_props in tools_config.get('grippers', {}).items():
        tool_names.append(tool_name)
        tool_types.append(tool_props['type'])
        motor_ids.append(str(tool_props['motor_id']))

    return [
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('aist_fastening_tools'), 'launch',
                'dynamixel_controllers.launch.py']),
            launch_arguments=[
                ('tool_names', ','.join(tool_names)),
                ('tool_types', ','.join(tool_types)),
                ('motor_ids',  ','.join(motor_ids)),
                ('usb_port',   tools_config['usb_port']),
                ('baud_rate',  str(tools_config['baud_rate'])),
                ('container',  [LaunchConfiguration('name'), '_container']),
                ('driver_ns',  [LaunchConfiguration('name'), '_driver'])])]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
