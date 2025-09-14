from launch                     import LaunchDescription
from launch.actions             import (IncludeLaunchDescription,
                                        OpaqueFunction)
from launch.substitutions       import (LaunchConfiguration, ThisLaunchFileDir,
                                        PathJoinSubstitution)
from launch_ros.substitutions   import FindPackageShare
from aist_bringup.launch_common import declare_launch_arguments

launch_arguments = [
    {
        'name':        'name',
        'default':     'precision_tool',
        'description': 'precision tool name'
    },
    {
        'name':        'collision',
        'default':     'false',
        'description': 'display collision mesh if true',
        'choices':     ['true', 'false', 'True', 'False']
    }
]

def launch_setup(context):
    return [
        IncludeLaunchDescription(
            PathJoinSubstitution([
                PathJoinSubstitution([FindPackageShare('aist_description'),
                                      'launch', 'display_part.launch.py'])]),
            launch_arguments=[
                ('name',            LaunchConfiguration('name')),
                ('properties_file', PathJoinSubstitution([
                                        FindPackageShare(
                                            'aist_precision_gripper'),
                                        'config',
                                        'precision_tool_properties.yaml'])),
                ('collision',       LaunchConfiguration('collision')),
                ('joint_gui',       'true')])]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
