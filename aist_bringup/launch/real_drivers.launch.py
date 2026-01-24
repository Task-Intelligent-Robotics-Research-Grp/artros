from launch                     import LaunchDescription
from launch.actions             import (IncludeLaunchDescription,
                                        OpaqueFunction, GroupAction)
from launch.substitutions       import LaunchConfiguration
from launch_ros.actions         import PushROSNamespace
from aist_bringup.launch_common import (declare_launch_arguments, load_config,
                                        get_arm_props, get_gripper_props)


launch_arguments = [
    {
        'name':        'config',
        'default':     'aist',
        'description': 'Name of the hardware configuration'
    },
]


def launch_setup(context):
    config  = load_config(context)
    actions = []
    for arm_name, arm_config in config.get('arms', {}).items():
        arm_props = get_arm_props(arm_config['type'])
        real_drivers_launch_file = arm_props.get('real_drivers_launch_file')
        if real_drivers_launch_file is not None:
            actions.append(
                GroupAction(
                    actions=[
                        PushROSNamespace(arm_name),
                        IncludeLaunchDescription(
                            real_drivers_launch_file,
                            launch_arguments=[
                                ('config', LaunchConfiguration('config')),
                                ('name',   arm_name)
                            ])
                    ]))
    for gripper_name, gripper_config in config.get('grippers', {}).items():
        gripper_props = get_gripper_props(gripper_config['type'])
        real_drivers_launch_file = gripper_props.get(
                                       'real_drivers_launch_file')
        if real_drivers_launch_file is not None:
            actions.append(
                IncludeLaunchDescription(
                    real_drivers_launch_file,
                    launch_arguments=[
                        ('name',   gripper_name)
                    ]))
    return actions

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
