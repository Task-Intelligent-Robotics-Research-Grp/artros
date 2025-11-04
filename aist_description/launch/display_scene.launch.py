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
        'name':        'config',
        'default':     'aist',
        'description': 'Configuration name of the scene'
    },
    {
        'name':        'scene',
        'default':     '',
        'description': 'Name of the scene'
    },
    {
        'name':        'sim',
        'default':     'false',
        'description': 'Use setting of gazebo simulation if true',
        'choices':     ['true', 'false', 'True', 'False']
    },
    {
        'name':        'joint_gui',
        'default':     'true',
        'description': 'Launch joint_state_publisher_gui if true',
        'choices':     ['true', 'false', 'True', 'False']
    }
]

def launch_setup(context):
    robot_description = ParameterValue(
                            Command([FindExecutable(name='xacro'),
                                     ' ',
                                     PathJoinSubstitution(
                                         [ThisLaunchFileDir(),
                                          '..', 'scenes', 'urdf',
                                          [LaunchConfiguration('config'),
                                           '_base_scene.urdf.xacro']]),
                                     ' scene:=',
                                     LaunchConfiguration('scene'),
                                     ' sim:=',
                                     LaunchConfiguration('sim')]),
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
                                                  'display_scene.rviz'])])]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
