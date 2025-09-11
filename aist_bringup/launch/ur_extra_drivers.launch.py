from launch                     import LaunchDescription
from launch.actions             import OpaqueFunction
from launch.conditions          import IfCondition
from launch.substitutions       import LaunchConfiguration
from launch_ros.actions         import Node
from aist_bringup.launch_common import declare_launch_arguments, load_config


launch_arguments = [
    {
        'name':        'config',
        'default':     'aist',
        'description': 'Name of the hardware configuration'
    },
    {
        'name':        'arm_name',
        'default':     'a_bot',
        'description': 'Name of the UR arm'
    },
]


def launch_setup(context):
    config        = load_config(context)
    arm_config    = config['arms'][LaunchConfiguration('arm_name') \
                                   .perform(context)]
    robot_ip      = arm_config['robot_ip']
    headless_mode = arm_config.get('headless_mode', False)

    return [
        Node(package='ur_robot_driver',
             executable='dashboard_client',
             name='dashboard_client',
             output='screen',
             emulate_tty=True,
             parameters=[{'robot_ip': robot_ip}]),
        Node(package='ur_robot_driver',
             executable='robot_state_helper',
             name='ur_robot_state_helper',
             output='screen',
             parameters=[{'headless_mode': headless_mode,
                          'robot_ip':      robot_ip}]),
        Node(condition=IfCondition(arm_config.get('use_tool_communication',
                                                  'false')),
             package='ur_robot_driver',
             executable='tool_communication.py',
             name='ur_tool_comm',
             output='screen',
             parameters=[
                 {'robot_ip':    robot_ip,
                  'tcp_port':    arm_config.get('tool_tcp_port', '54321'),
                  'device_name': arm_config.get('tool_device_name',
                                                '/tmp/ttyUR')}]),
        Node(package='ur_robot_driver',
             executable='urscript_interface',
             parameters=[{'robot_ip': robot_ip}],
             output='screen'),
        Node(package='ur_robot_driver',
             executable='controller_stopper_node',
             name='controller_stopper',
             output='screen',
             emulate_tty=True,
             parameters=['headless_mode': headless_mode,
                         'joint_controller_active':
                         arm_config.get('joint_controller_active', True),
                         'consistent_controllers':
                         ['joint_state_broadcaster'] + \
                         arm_config.get('consistent_controllers', []) + \
                         arm_config.get('extra_consistent_controllers', [])])]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
