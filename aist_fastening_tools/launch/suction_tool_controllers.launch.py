from launch                            import LaunchDescription
from launch.actions                    import (SetLaunchConfiguration,
                                               OpaqueFunction)
from launch.substitutions              import (LaunchConfiguration,
                                               PathJoinSubstitution)
from launch_ros.actions         import Node, LoadComposableNodes
from launch_ros.descriptions    import ComposableNode
from aist_bringup.launch_common import declare_launch_arguments

launch_arguments = [
    {
        'name':        'tool_names',
        'default':     'screw_tool_m3,screw_tool_m4,suction_tool,base_fixture',
        'description': 'list of tool names'
    },
    {
        'name':        'digital_in_ports',
        'default':     '0,1,2,-1',
        'description': 'list of IDs of the digital IN ports'
    },
    {
        'name':        'digital_out_ports_suck',
        'default':     '0,2,4,6',
        'description': 'list of IDs of the digital OUT ports for suck'
    },
    {
        'name':        'digital_out_ports_blow',
        'default':     '1,3,5,-1',
        'description': 'list of ID of the digital OUT port for blow'
    },
    {
        'name':        'joint_names',
        'default':     ',,,base_fixture_piston_joint',
        'description': 'list of ID of the digital OUT port for blow'
    },
    {
        'name':        'container',
        'default':     'screw_tools_container',
        'description': 'name of the component container'
    },
    {
        'name':        'driver_ns',
        'default':     'b_bot_io_and_status_controller',
        'description': 'namespace of the IO controller of the UR arm'
    },
    {
        'name':        'log_level',
        'default':     'info',
        'description': 'debug log level',
        'choices':     ['debug', 'info', 'warn', 'error', 'fatal']
    },
    {
        'name':        'output',
        'default':     'both',
        'description': 'pipe node output',
        'choices':     ['screen', 'log', 'both']
    }
]

def launch_setup(context):
    plugin = 'aist_fastening_tools::SuctionToolController'
    composable_nodes = []
    for tool_name, din_port, dout_port_suck, dout_port_blow, joint_name \
          in zip(LaunchConfiguration('tool_names')\
                 .perform(context).split(','),
                 LaunchConfiguration('digital_in_ports')\
                 .perform(context).split(','),
                 LaunchConfiguration('digital_out_ports_suck')\
                 .perform(context).split(','),
                 LaunchConfiguration('digital_out_ports_blow')\
                 .perform(context).split(','),
                 LaunchConfiguration('joint_names')\
                 .perform(context).split(',')):
        composable_nodes.append(
            ComposableNode(
                name=tool_name + '_controller',
                package='aist_fastening_tools',
                plugin=plugin,
                parameters=[
                    {'driver_ns':             LaunchConfiguration('driver_ns'),
                     'digital_in_port':       digital_in_port,
                     'digital_out_port_suck': digital_out_port_suck,
                     'digital_out_port_blow': digital_out_port_blow,
                     'joint_name':            joint_name}
                ],
                extra_arguments=[{'use_intra_process_comms': True}]))

    return [
        Node(name=LaunchConfiguration('container'),
             package='rclcpp_components',
             executable='component_container_mt',
             output=LaunchConfiguration('output'),
             arguments=['--ros-args', '--log-level',
                        LaunchConfiguration('log_level')]),
        LoadComposableNodes(
            target_container=LaunchConfiguration('container'),
            composable_node_descriptions=composable_nodes)
    ]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
