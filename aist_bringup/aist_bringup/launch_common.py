import os, yaml
from launch.actions                    import DeclareLaunchArgument
from launch.substitutions              import (LaunchConfiguration,
                                               PathJoinSubstitution)
from launch_ros.substitutions          import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile


ARM_PROPS = {
    'UR':
    {
        'update_rate':               125,
        'controllers_template':      PathJoinSubstitution(
                                         [FindPackageShare('aist_bringup'),
                                          'config', 'templates',
                                          'ur_controllers.yaml']),
        'gz_controllers_template':   PathJoinSubstitution(
                                         [FindPackageShare('aist_bringup'),
                                          'config', 'templates',
                                          'ur_gz_controllers.yaml']),
        'extra_drivers_launch_file': PathJoinSubstitution(
                                         [FindPackageShare('aist_bringup'),
                                          'launch', 'inc',
                                          'ur_extra_drivers.launch.py'])
    },
    'URe':
    {
        'update_rate':               500,
        'controllers_template':      PathJoinSubstitution(
                                         [FindPackageShare('aist_bringup'),
                                          'config', 'templates',
                                          'ur_controllers.yaml']),
        'gz_controllers_template':   PathJoinSubstitution(
                                         [FindPackageShare('aist_bringup'),
                                          'config', 'templates',
                                          'ur_gz_controllers.yaml']),
        'extra_drivers_launch_file': PathJoinSubstitution(
                                         [FindPackageShare('aist_bringup'),
                                          'launch', 'inc',
                                          'ur_extra_drivers.launch.py'])
    },
}

GRIPPER_PROPS = {
    'RobotiqGripper':
    {
        'gz_controllers_template':   PathJoinSubstitution(
                                         [FindPackageShare('aist_bringup'),
                                          'config', 'templates',
                                          'gripper_controllers.yaml']),
        'extra_drivers_launch_file': PathJoinSubstitution(
                                         [FindPackageShare('aist_bringup'),
                                          'launch', 'inc',
                                          'robotiq_drivers.launch.py'])
    },
    'PrecisionGripper':
    {
        'extra_drivers_launch_file': PathJoinSubstitution(
                                         [FindPackageShare('aist_bringup'),
                                          'launch', 'inc',
                                          'precision_gripper_drivers.launch.py'])
    },
    'ScrewTool':
    {
    },
    'SuctionGripper':
    {
        'extra_drivers_launch_file': PathJoinSubstitution(
                                         [FindPackageShare('aist_bringup'),
                                          'launch', 'inc',
                                          'suction_gripper_drivers.launch.py'])
    },
}

CAMERA_PROPS = {
    'PhoXiCamera':
    {
        'launch_file': PathJoinSubstitution(
                           [FindPackageShare('aist_phoxi_camera'), 'launch',
                            'launch.py']),
        'key_of_id':   'id',
        'cloud_topic': 'pointcloud',
        'depth_topic': 'depth_map',
        'cinfo_topic': 'camera_info',
        'color_topic': 'texture'
    },
    'RealsenseCamera':
    {
        'launch_file': PathJoinSubstitution(
                           [FindPackageShare('realsense2_camera'), 'launch',
                            'launch.py']),
        'key_of_id':   'serial_no',
        'cloud_topic': 'depth/color/points',
        'depth_topic': 'aligned_depth_to_color/image_raw',
        'cinfo_topic': 'aligned_depth_to_color/camera_info',
        'color_topic': 'color/image_raw'
    },
    'CodedLightRealsenseCamera':
    {
        'launch_file': PathJoinSubstitution(
                           [FindPackageShare('realsense2_camera'), 'launch',
                            'launch.py']),
        'key_of_id':   'serial_no',
        'cloud_topic': 'depth/color/points',
        'depth_topic': 'aligned_depth_to_color/image_raw',
        'cinfo_topic': 'aligned_depth_to_color/camera_info',
        'color_topic': 'color/image_raw'
    },
}

def get_arm_props(arm_type):
    return ARM_PROPS[arm_type]

def get_gripper_props(gripper_type):
    return GRIPPER_PROPS[gripper_type]

def get_camera_props(camera_type):
    return CAMERA_PROPS[camera_type]

def instantiate_file(context, template_file, instantiated_file, append=False):
    # We must extend lifetime of the ParameterFile object by keeping it
    # in a variable or the temporary file created by evaluating it
    # will be immediately erased by the destructor of ParameterFile.
    parameter_file = ParameterFile(template_file, allow_substs=True)
    if append:
        with open(parameter_file.evaluate(context)) as fin:
            content = fin.read()
            with open(instantiated_file, mode='a') as fout:
                fout.write(content)
    else:
        # Rename the created file to prevent from being erased.
        os.rename(parameter_file.evaluate(context), instantiated_file)
    parameter_file
    return instantiated_file

def load_config(context):
    config_file = PathJoinSubstitution([FindPackageShare('aist_bringup'),
                                        'config',
                                        [LaunchConfiguration('config'),
                                         '.yaml']])
    with open(config_file.perform(context), 'r') as f:
        config = yaml.safe_load(f)
    return config

def declare_launch_arguments(args):
    return [DeclareLaunchArgument(arg['name'],
                                  default_value=arg.get('default'),
                                  description=arg.get('description'),
                                  choices=arg.get('choices')) \
            for arg in args]

def set_configurable_parameters(args):
    return {arg['name']: LaunchConfiguration(arg['name']) for arg in args}
