from launch                   import LaunchDescription
from launch.actions           import (DeclareLaunchArgument,
                                      IncludeLaunchDescription, OpaqueFunction)
from launch.substitutions     import (LaunchConfiguration, ThisLaunchFileDir,
                                      PathJoinSubstitution, EqualsSubstitution)
from launch_ros.substitutions import FindPackageShare

launch_arguments = [
    {'name':        'name',
     'default':     'precision_tool',
     'description': 'precision tool name'},
    {'name':        'collision',
     'default':     'false',
     'description': 'display collision mesh if true'}]

def declare_launch_arguments(args):
    return [DeclareLaunchArgument(arg['name'],
                                  default_value=arg['default'],
                                  description=arg['description']) \
            for arg in args]

def launch_setup(context):
    launch_dir = PathJoinSubstitution([FindPackageShare('aist_description'),
                                       'launch'])
    prop_file  = PathJoinSubstitution([FindPackageShare(
                                           'aist_precision_gripper'),
                                       'config',
                                       'precision_tool_properties.yaml'])


    return [
        IncludeLaunchDescription(
            PathJoinSubstitution([launch_dir, 'display_part.launch.py']),
            launch_arguments={
                'name':            LaunchConfiguration('name'),
                'properties_file': prop_file,
                'collision':       LaunchConfiguration('collision'),
            }.items())]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
