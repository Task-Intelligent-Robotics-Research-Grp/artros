from launch                            import LaunchDescription
from launch.actions                    import OpaqueFunction
from launch.substitutions              import (LaunchConfiguration,
                                               PathJoinSubstitution)
from launch_ros.actions                import Node
from launch_ros.substitutions          import FindPackageShare
from moveit_configs_utils              import MoveItConfigsBuilder
from launch_ros.parameter_descriptions import ParameterFile
from aist_bringup.launch_common        import declare_launch_arguments


launch_arguments = [
    {
        'name':        'name',
        'default':     'interactive',
        'description': 'Name of the client'
    },
    {
        'name':        'config',
        'default':     'aist',
        'description': 'Name of the hardware configuration'
    },
    {
        'name':        'setting_file',
        'default':     PathJoinSubstitution([
                           FindPackageShare('aist_routines'), 'config',
                           [LaunchConfiguration('name'), '.yaml']]),
        'description': 'Name of the hardware configuration'
    },
    {
        'name':        'sim',
        'default':     'false',
        'description': 'Use simulation time if true',
        'choices':     ['true', 'false', 'True', 'False']
    },
]

def launch_setup(context):
    return [Node(name=LaunchConfiguration('name'),
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
                 parameters=[
                     ParameterFile(LaunchConfiguration('setting_file'),
                                   allow_substs=True),
                     {'config_file':
                      PathJoinSubstitution([
                          FindPackageShare('aist_bringup'), 'config',
                          [LaunchConfiguration('config'), '.yaml']]),
                      'use_sim_time': LaunchConfiguration('sim'),
                     }
                 ],
                 prefix=['xterm -fn 7x14 -sb -geometry 80x60 -e'],
                 output='screen')]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
