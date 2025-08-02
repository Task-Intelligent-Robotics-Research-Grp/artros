from launch                            import LaunchDescription
from launch.actions                    import (DeclareLaunchArgument,
                                               OpaqueFunction)
from launch.substitutions              import (LaunchConfiguration,
                                               ThisLaunchFileDir,
                                               PathJoinSubstitution,
                                               EqualsSubstitution,
                                               IfElseSubstitution, Command)
from launch.conditions                 import IfCondition
from launch_ros.actions                import Node
from launch_ros.parameter_descriptions import ParameterValue

launch_arguments = [
    {'name':        'name',
     'default':     'base',
     'description': 'part name'},
    {'name':        'properties_file',
     'default':     '',
     'description': 'path to YAML file containing part properties'},
    {'name':        'collision',
     'default':     'false',
     'description': 'display collision mesh if true'},
    {'name':        'rvizconfig',
     'default':     'display_part.rviz',
     'description': 'path to configuration file of rviz'}]

def declare_launch_arguments(args):
    return [DeclareLaunchArgument(arg['name'],
                                  default_value=arg['default'],
                                  description=arg['description']) \
            for arg in args]

def launch_setup(context):
    urdf_file = PathJoinSubstitution([ThisLaunchFileDir(), '..', 'parts',
                                      'urdf', 'object.urdf'])
    prop_file = IfElseSubstitution(
                    EqualsSubstitution(
                        LaunchConfiguration('properties_file'), ''),
                    PathJoinSubstitution(
                        [ThisLaunchFileDir(), '..', 'parts', 'config',
                         'parts_properties.yaml']),
                    LaunchConfiguration('properties_file'))
    print(prop_file.perform(context))
    robot_description = ParameterValue(
                            Command(['xacro ', urdf_file,
                                     ' name:=', LaunchConfiguration('name'),
                                     ' properties_file:=', prop_file,
                                     ' collision:=',
                                     LaunchConfiguration('collision')]),
                            value_type=str)
    return [Node(package='robot_state_publisher',
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
