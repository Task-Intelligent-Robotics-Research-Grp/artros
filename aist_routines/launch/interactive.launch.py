from pathlib                     import Path
from launch                      import LaunchDescription
from launch.actions              import OpaqueFunction
from launch.substitutions        import LaunchConfiguration
from launch_ros.actions          import Node
from moveit_configs_utils        import MoveItConfigsBuilder
from aist_bringup.launch_common  import (declare_launch_arguments,
                                         set_configurable_parameters)


launch_arguments = [
    {
        'name':        'config',
        'default':     'aist',
        'description': 'Name of the hardware configuration'
    },
]

parameter_arguments = [
    {
        'name':        'reference_frame',
        'default':     'workspace_center',
        'description': 'reference frame for MoveIt'
    },
    {
        'name':        'eef_step',
        'default':     '0.005',
        'description': 'reference frame for MoveIt'
    }
]

def launch_setup(context, param_args):
    return [Node(name='interactive',
                 package='aist_routines',
                 executable='interactive',
                 # parameters=[
                 #     moveit_configs.to_dict(),
                 #     {'robot_name':
                 #      [LaunchConfiguration('config'), '_base_scene'],
                 #      'moveit_config_package':
                 #      [LaunchConfiguration('config'), '_moveit_config']},
                 #     # set_configurable_parameters(param_args)
                 # ],
#                 prefix=['xterm -fn 7x14 -e'],
                 output='screen')]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments +
                                                      parameter_arguments) + \
                             [OpaqueFunction(function=launch_setup,
                                             args=[parameter_arguments])])
