from launch                     import LaunchDescription
from launch.actions             import (IncludeLaunchDescription,
                                        OpaqueFunction, GroupAction)
from launch.substitutions       import (LaunchConfiguration, ThisLaunchFileDir,
                                        PathJoinSubstitution)
from launch_ros.substitutions   import FindPackageShare
from launch_ros.actions         import PushROSNamespace
from aist_bringup.launch_common import declare_launch_arguments


launch_arguments = [
    {
        'name':        'robot_names',
        'default':     'a_bot,b_bot',
        'description': 'list of robot names'
    },
]

def launch_setup(context):
    actions = [
        IncludeLaunchDescription(
            PathJoinSubstitution([ThisLaunchFileDir(), 'base.launch.py']))
    ]
    for robot_name in LaunchConfiguration('robot_names').perform(context) \
                                                        .split(','):
        actions.append(
            GroupAction(
                actions=[
                    PushROSNamespace(robot_name),
                    IncludeLaunchDescription(
                        PathJoinSubstitution(
                            [FindPackageShare('cuda_feature_tracker_3d'),
                             'launch', 'launch.py']),
                        launch_arguments=[
                            ('robot_name',  robot_name),
                            ('camera_type', 'RealsenseCamera'),
                        ]),
                ]))
    return actions

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
