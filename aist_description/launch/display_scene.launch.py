from launch               import LaunchDescription
from launch.actions       import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (LaunchConfiguration, ThisLaunchFileDir,
                                  PathJoinSubstitution, StringJoinSubstitution,
                                  Command)
from launch.conditions    import IfCondition
from launch_ros.actions   import Node

launch_arguments = [
    {'name':        'config',
     'default':     'aist',
     'description': 'configuration name of the scene'},
    {'name':        'scene',
     'default':     '',
     'description': 'name of the scene'},
    {'name':        'rvizconfig',
     'default':     'display_scene.rviz',
     'description': 'path to configuration file of rviz'},
    {'name':        'joint_gui',
     'default':     'true',
     'description': 'launch joint_state_publisher if true'}]

def declare_launch_arguments(args):
    return [DeclareLaunchArgument(arg['name'],
                                  default_value=arg['default'],
                                  description=arg['description']) \
            for arg in args]

def launch_setup(context):
    urdf_path = PathJoinSubstitution(
                    [ThisLaunchFileDir(), '..', 'scenes', 'urdf',
                     StringJoinSubstitution([LaunchConfigurarion('config'),
                                             '_base_scene.urdf.xacro'])])
    robot_description = ParameterValue(Command(['xacro ', urdf_path,
                                                ' scene:=',
                                                LaunchConfiguration('scene')]),
                                       value_type=str)
    return [Node(pacakge='robot_state_publisher',
                 executable='robot_state_publisher',
                 parameters=[{'robot_description': robot_description}]),
            Node(package='joint_state_publisher',
                 executable='joint_state_publisher',
                 condition=IfCondition(LaunchConfiguration('joint_gui'))),
            Node(name='rviz', package='rviz2', executable='rviz2',
                 output='screen',
                 arguments=['-d',
                            PathJoinSubstitution([
                                ThisLaunchFileDir(),
                                LaunchConfiguration('rvizconfig')])])]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
