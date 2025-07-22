from launch               import LaunchDescription
from launch.actions       import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (LaunchConfiguration, ThisLaunchFileDir,
                                  PathJoinSubstitution, StringJoinSubstitution,
                                  Command)
from launch.conditions    import IfCondition
from launch_ros.actions   import Node

launch_arguments = [
    {'name':        'name',
     'default':     'base',
     'description': 'part name'},
    {'name':        'collision',
     'default':     'false',
     'description': 'display collision mesh if true'},
    {'name':        'properties_file',
     'default':     'parts_properties.yaml',
     'description': 'YAML file name storing parts properties'},
    {'name':        'rvizconfig',
     'default':     'display_part.rviz',
     'description': 'path to configuration file of rviz'}]

def declare_launch_arguments(args):
    return [DeclareLaunchArgument(arg['name'],
                                  default_value=arg['default'],
                                  description=arg['description']) \
            for arg in args]

def launch_setup(context):
    urdf_path = PathJoinSubstitution([ThisLaunchFileDir(), '..', 'parts',
                                      'urdf', 'object.urdf'])
    robot_description = ParameterValue(
                            Command(['xacro ', urdf_path,
                                     ' name:=',
                                     LaunchConfiguration('name'),
                                     ' properties_file=',
                                     LuanchConfiguration('properties_file'),
                                     ', collision:=',
                                     LaunchConfiguration('collision')]),
                            value_type=str)
    return [Node(pacakge='robot_state_publisher',
                 executable='robot_state_publisher',
                 parameters=[{'robot_description': robot_description}]),
            Node(name='rviz', package='rviz2', executable='rviz2',
                 output='screen',
                 arguments=['-d',
                            PathJoinSubstitution([
                                ThisLaunchFileDir(),
                                LaunchConfiguration('rvizconfig')])])]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
