import os, yaml
from launch                            import LaunchDescription
from launch.actions                    import (SetLaunchConfiguration,
                                               DeclareLaunchArgument,
                                               IncludeLaunchDescription,
                                               OpaqueFunction,
                                               GroupAction)
from launch.conditions                 import IfCondition, UnlessCondition
from launch.substitutions              import (Command, FindExecutable,
                                               LaunchConfiguration,
                                               ThisLaunchFileDir,
                                               PathJoinSubstitution,
                                               IfElseSubstitution)
from launch_ros.actions                import Node
from launch_ros.substitutions          import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile
from aist_bringup.launch_common        import (declare_launch_arguments,
                                               load_config, get_arm_props,
                                               get_gripper_props,
                                               instantiate_config_file)

launch_arguments = [
    {
        'name':        'config',
        'default':     'aist',
        'description': 'Name of the hardware configuration'
    },
]


def launch_setup(context):
    config = load_config(context)
    for camera_name, camera_config in config['cameras'].items():
        camera_props = get_camera_props(camera_config['type'])
        SetLaunchConfiguration('camera_name',
                               value=camera_name).execute(context)
        SetLaunchConfiguration('color_topic',
                               value=camera_props['name).execute(context)


def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
