from launch                            import LaunchDescription
from launch.actions                    import OpaqueFunction
from launch.substitutions              import (LaunchConfiguration,
                                               ThisLaunchFileDir,
                                               PathJoinSubstitution,
                                               Command, FindExecutable)
from launch.conditions                 import IfCondition
from launch_ros.actions                import Node
from launch_ros.parameter_descriptions import ParameterValue
from aist_bringup.launch_common        import declare_launch_arguments

launch_arguments = [
    {
        'name':        'name',
        'default':     'base',
        'description': 'Part name'
    },
    {
        'name':        'properties_file',
        'default':     PathJoinSubstitution(
                           [ThisLaunchFileDir(), '..', 'parts', 'config',
                            'parts_properties.yaml']),
        'description': 'Absolute path to YAML file containing part properties'
    },
    {
        'name':        'collision',
        'default':     'false',
        'description': 'Display collision mesh if true',
        'choices':     ['true', 'false', 'True', 'False']
    },
    {
        'name':        'joint_gui',
        'default':     'false',
        'description': 'Launch joint_state_publisher if true',
        'choices':     ['true', 'false', 'True', 'False']
    }
]

def launch_setup(context):
    robot_description = ParameterValue(
                            Command([FindExecutable(name='xacro'),
                                     ' ',
                                     PathJoinSubstitution(
                                         [ThisLaunchFileDir(), '..', 'parts',
                                          'urdf', 'object.urdf']),
                                     ' name:=', LaunchConfiguration('name'),
                                     ' properties_file:=',
                                     LaunchConfiguration('properties_file'),
                                     ' collision:=',
                                     LaunchConfiguration('collision')]),
                            value_type=str)
    return [Node(package='robot_state_publisher',
                 executable='robot_state_publisher',
                 parameters=[{'robot_description': robot_description}]),
            Node(package='joint_state_publisher_gui',
                 executable='joint_state_publisher_gui',
                 condition=IfCondition(LaunchConfiguration('joint_gui'))),
            Node(name='rviz', package='rviz2', executable='rviz2',
                 output='screen',
                 arguments=['-d',
                            PathJoinSubstitution([ThisLaunchFileDir(),
                                                  'display_part.rviz'])])]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
