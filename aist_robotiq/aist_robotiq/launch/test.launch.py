from launch                            import LaunchDescription
from launch.actions                    import (OpaqueFunction,
                                               IncludeLaunchDescription)
from launch.substitutions              import (Command, FindExecutable,
                                               LaunchConfiguration,
                                               ThisLaunchFileDir,
                                               PathJoinSubstitution,
                                               IfElseSubstitution,
                                               EqualsSubstitution)
from launch_ros.substitutions          import FindPackageShare
from launch_ros.actions                import Node
from launch_ros.parameter_descriptions import ParameterValue
from aist_bringup.launch_common        import declare_launch_arguments

launch_arguments = [
    {
        'name':        'gripper_name',
        'default':     'robotiq_85',
        'description': 'name of the gripper',
        'choices':     ['robotiq_85', 'robotiq_140', 'robotiq_hande',
                        'robotiq_3f', 'robotiq_epick'],
    },
]

def launch_setup(context):
    gripper_type = IfElseSubstitution(
                       EqualsSubstitution(
                           LaunchConfiguration('gripper_name'), 'robotiq_epick'),
                       'RobotiqSuction', 'RobotiqGripper')
    client_type = IfElseSubstitution(
                      EqualsSubstitution(gripper_type, 'RobotiqSuction'),
                      'suction', 'gripper')
    robot_description = ParameterValue(
                            Command([
                                FindExecutable(name='xacro'), ' ',
                                PathJoinSubstitution([
                                    FindPackageShare('aist_robotiq'), 'urdf',
                                    [LaunchConfiguration('gripper_name'),
                                     '_gripper.urdf']
                                ])
                            ]),
                            value_type=str)
    return [
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             parameters=[
                 {'robot_description': robot_description}
             ],
             output='screen'),
        IncludeLaunchDescription(
            PathJoinSubstitution([ThisLaunchFileDir(), 'launch.py']),
            launch_arguments=[
                ('gripper_names',  LaunchConfiguration('gripper_name')),
                ('gripper_types',  gripper_type),
                ('driver_ns',      [LaunchConfiguration('gripper_name'),
                                    '_driver']),
            ]),
        Node(name=['test_', client_type, '_client'],
             package='aist_robotiq',
             executable=['test_', client_type, '_client.py'],
             parameters=[{'gripper_name': LaunchConfiguration('gripper_name')}],
             prefix=['xterm -fn 7x14 -e'],
             output='screen'),
        Node(name='rviz', package='rviz2', executable='rviz2',
             output='screen',
             arguments=[
                 '-d',
                 PathJoinSubstitution([FindPackageShare('aist_robotiq'),
                                       'launch', 'aist_robotiq.rviz'])
             ]),
    ]

def generate_launch_description():
    return LaunchDescription(declare_launch_arguments(launch_arguments) + \
                             [OpaqueFunction(function=launch_setup)])
