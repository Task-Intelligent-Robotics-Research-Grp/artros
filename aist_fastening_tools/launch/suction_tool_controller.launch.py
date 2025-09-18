from launch                            import LaunchDescription
from launch.actions                    import OpaqueFunction
from launch.substitutions              import (LaunchConfiguration,
                                               PathJoinSubstitution)
from launch_ros.actions                import Node
from launch_ros.substitutions          import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile
from aist_bringup.launch_common        import declare_launch_arguments

launch_arguments = [
    {
        'name':        'name',
        'default':     'suction_tool',
        'description': 'name of the suction tool'
    },
    {
        'name':        'driver_ns',
        'default':     'b_bot_io_and_status_controller',
        'description': 'namespace of the IO controller of the UR arm'
    },
    {
        'name':        'digital_in_port',
        'default':     '2',
        'description': 'ID of the digital IN port for the tool state'
    },
    {
        'name':        'digital_out_port_suck',
        'default':     '4',
        'description': 'ID of the digital OUT port for suck'
    },
    {
        'name':        'digital_out_port_blow',
        'default':     '5',
        'description': 'ID of the digital OUT port for blow'
    },
    {
        'name':        'joint_name',
        'default':     '""',
        'description': 'name of the joint if exists'
    },
    {
        'name':        'log_level',
        'default':     'info',
        'description': 'debug log level',
        'choices':     ['debug', 'info', 'warn', 'error', 'fatal']
    }
]

def launch_setup(context):
    param_file = ParameterFile(PathJoinSubstitution(
                                   [FindPackageShare('aist_fastening_tools'),
                                    'config', 'suction_tool_controller.yaml']),
                               allow_substs=True)
    return [Node(name=[LaunchConfiguration('name'), '_controller'],
                 package='aist_fastening_tools',
                 executable='suction_tool_controller.py',
                 parameters=[param_file],
                 output='screen',
                 arguments=['--ros-args', '--log-level',
                            LaunchConfiguration('log_level')])]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
