from launch                            import LaunchDescription
from launch.actions                    import (IncludeLaunchDescription,
                                               OpaqueFunction)
from launch.substitutions              import (LaunchConfiguration,
                                               PathJoinSubstitution,
                                               Command, FindExecutable)
from launch_ros.substitutions          import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from aist_bringup.launch_common        import declare_launch_arguments

launch_arguments = [
    {
        'name':        'config',
        'default':     'aist',
        'description': 'Configuration name of the scene'
    },
    {
        'name':        'scene',
        'default':     '',
        'description': 'Name of the scene'
    },
    {
        'name':        'sim',
        'default':     'false',
        'description': 'Use setting of gazebo simulation if true',
        'choices':     ['true', 'false', 'True', 'False']
    },
    {
        'name':        'param_file',
        'default':     PathJoinSubstitution([
                           FindPackageShare('aist_visualization'), 'config',
                           'default.yaml']),
        'description': 'abolute path to YAML file for configuration'
    },
]

def launch_setup(context):
    robot_description = ParameterValue(
                            Command(
                                [FindExecutable(name='xacro'),
                                 ' ',
                                 PathJoinSubstitution(
                                     [FindPackageShare('aist_visualization'),
                                      'urdf', 'test_arm.urdf'])]),
                            value_type=str)
    return [
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare('aist_description'), 'launch',
                 'display_scene.launch.py'])),
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare('aist_visualization'), 'launch',
                 'launch.py'])),
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare('nep_bridge'), 'launch', 'launch.py'])),
    ]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
