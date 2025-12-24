from launch                            import LaunchDescription
from launch.actions                    import (IncludeLaunchDescription,
                                               OpaqueFunction)
from launch.substitutions              import (LaunchConfiguration,
                                               PathJoinSubstitution,
                                               Command, FindExecutable)
from launch_ros.substitutions          import FindPackageShare
from launch_ros.actions                import Node
from launch_ros.parameter_descriptions import ParameterValue
from aist_bringup.launch_common        import declare_launch_arguments

launch_arguments = [
    {
        'name':        'param_file',
        'default':     PathJoinSubstitution([
                           FindPackageShare('aist_visualization'), 'config',
                           'default.yaml']),
        'description': 'abolute path to YAML file for configuration'
    },
]

def launch_setup(context):
    robot_description = ParameterValue(
                            Command(
                                [FindExecutable(name='xacro'),
                                 ' ',
                                 PathJoinSubstitution(
                                     [FindPackageShare('aist_visualization'),
                                      'urdf', 'test_arm.urdf'])]),
                            value_type=str)
    return [
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        Node(package='joint_state_publisher_gui',
             executable='joint_state_publisher_gui'),
        Node(name='rviz', package='rviz2', executable='rviz2', output='screen',
             arguments=['-d',
                        PathJoinSubstitution([
                            FindPackageShare('aist_visualization'),
                            'launch', 'test.rviz'])]),
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare('aist_visualization'), 'launch',
                 'launch.py'])),
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare('nep_bridge'), 'launch', 'launch.py'])),
    ]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
