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
        'default':     'screw_tool_m4',
        'description': 'Name of the screw tool'
    },
]

usb_ports = {}

def launch_setup(context):
    config         = load_config(context)
    gripper_config = config['grippers'][LaunchConfiguration('name') \
                                       .perform(context)]
    usb_port = gripper_config['usb_port']
    if usb_port not in usb_ports:
        usb_ports[usb_port] =

    return [
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('aist_fastening_tools'),
                                  'launch',
                                  'suction_tool_controller.launch.py']),
            launch_arguments=[
                ('name',            LaunchConfiguration('name')),
                ('driver_ns',       gripper_config['driver_ns']),
                ('digital_in_port', gripper_config.get('digital_in_port', -1)),
                ('digital_out_port_suck',
                 gripper_config.get('digital_out_port_suck', -1)),
                ('digital_out_port_blow',
                 gripper_config.get('digital_out_port_blow', -1)),
                ('joint_name',      gripper_config.get('joint_name', ''))
            ])]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
