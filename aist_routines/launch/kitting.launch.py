from launch                     import LaunchDescription
from launch.actions             import (IncludeLaunchDescription,
                                        OpaqueFunction)
from launch.substitutions       import (LaunchConfiguration, ThisLaunchFileDir,
                                        PathJoinSubstitution)
from launch_ros.substitutions   import FindPackageShare


def launch_setup(context):
    return [
        IncludeLaunchDescription(
            PathJoinSubstitution([ThisLaunchFileDir(), 'base.launch.py'])),
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare('aist_graspability'), 'launch',
                 'launch.py']),
            launch_arguments=[
                ('name',        'graspability'),
                ('camera_name', 'a_motioncam'),
                ('camera_type', 'PhoXiCamera'),
                ('param_file',  LaunchConfiguration('settings_file')),
            ]),
    ]

def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
