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
                [FindPackageShare('cuda_feature_tracker_3d'),
                 'launch', 'launch.py']),
            launch_arguments=[
                ('robot_name',  'b_bot'),
                ('camera_type', 'RealsenseCamera'),
            ]),
    ]

def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
