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
        'default':     'suction_tools',
        'description': 'Name of the Dynamixel device group'
    },
]

def launch_setup(context):
    config       = load_config(context)
    tools_config = config['grippers'][LaunchConfiguration('name') \
                                      .perform(context)]
    tool_names      = []
    din_ports       = []
    dout_ports_suck = []
    dout_ports_blow = []
    joint_names     = []
    for tool_name, tool_props in tools_config['tools'].items():
        tool_names.append(tool_name)
        din_ports.append(str(tool_props.get('digital_in_port', -1)))
        dout_ports_suck.append(str(tool_props.get('digital_out_port_suck',-1)))
        dout_ports_blow.append(str(tool_props.get('digital_out_port_blow',-1)))
        joint_names.append(tool_props.get('joint_name', ''))

    return [
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('aist_fastening_tools'), 'launch',
                'suction_tool_controllers.launch.py']),
            launch_arguments=[
                ('tool_names',              ','.join(tool_names)),
                ('digital_in_ports',        ','.join(din_ports)),
                ('digital_out_ports_suck',  ','.join(dout_ports_suck)),
                ('digital_out_ports_blow',  ','.join(dout_ports_blow)),
                ('joint_names',             ','.join(joint_names)),
                ('container',  [LaunchConfiguration('name'), '_container']),
                ('driver_ns',  tools_config['driver_ns'])])]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
