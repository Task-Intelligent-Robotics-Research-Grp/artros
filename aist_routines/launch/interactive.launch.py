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
        'name':        'task',
        'default':     'base',
        'description': 'Name of the client',
        'choices':     ['base', 'assembly', 'kitting']
    },
    {
        'name':        'config',
        'default':     'aist',
        'description': 'Name of the hardware configuration'
    },
    {
        'name':        'param_file',
        'default':     PathJoinSubstitution([
                           FindPackageShare('aist_routines'), 'config',
                           'interactive.yaml']),
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
    task_name = ['interactive_', LaunchConfiguration('task')]

    return [Node(name=task_name,
                 package='aist_routines',
                 executable=task_name,
                 parameters=[
                     ParameterFile(LaunchConfiguration('param_file'),
                                   allow_substs=True),
                     {'config_file':
                      PathJoinSubstitution([
                          FindPackageShare('aist_bringup'), 'config',
                          [LaunchConfiguration('config'), '.yaml']]),
                      'use_sim_time': LaunchConfiguration('sim'),
                     }
                 ],
                 prefix=['gnome-terminal --tab --wait --'],
                 output='screen')]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
