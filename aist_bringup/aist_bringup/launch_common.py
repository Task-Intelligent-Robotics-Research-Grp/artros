import os, yaml
from launch.actions                    import DeclareLaunchArgument
from launch.substitutions              import (LaunchConfiguration,
                                               PathJoinSubstitution)
from launch_ros.substitutions          import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile
from aist_utility.fileio               import filepath_from_url


DEVICE_PROPS = {
    'UR':
    {
        'update_rate':              125,
        'controllers_template':     PathJoinSubstitution(
                                         [FindPackageShare('aist_bringup'),
                                          'config', 'templates',
                                          'ur_controllers.yaml']),
        'real_drivers_launch_file': PathJoinSubstitution(
                                        [FindPackageShare('aist_bringup'),
                                         'launch', 'inc',
                                         'ur_real_drivers.launch.py']),
        'ros2_control_file':        PathJoinSubstitution(
                                        [FindPackageShare('aist_bringup'),
                                         'urdf',
                                         'ur.ros2_control.urdf.xacro']),
    },
    'URe':
    {
        'update_rate':              500,
        'controllers_template':     PathJoinSubstitution(
                                        [FindPackageShare('aist_bringup'),
                                         'config', 'templates',
                                         'ur_controllers.yaml']),
        'real_drivers_launch_file': PathJoinSubstitution(
                                        [FindPackageShare('aist_bringup'),
                                         'launch', 'inc',
                                         'ur_real_drivers.launch.py']),
        'ros2_control_file':        PathJoinSubstitution(
                                        [FindPackageShare('aist_bringup'),
                                         'urdf',
                                         'ur.ros2_control.urdf.xacro']),
    },
    'LBR':
    {
        'update_rate':              100,
        'controllers_template':     PathJoinSubstitution(
                                        [FindPackageShare('aist_bringup'),
                                         'config', 'templates',
                                         'lbr_controllers.yaml']),
        'ros2_control_file':        PathJoinSubstitution(
                                        [FindPackageShare('aist_bringup'),
                                         'urdf',
                                         'lbr.ros2_control.urdf.xacro']),
    },

    'RobotiqDevices':
    {
        'real_drivers_launch_file': PathJoinSubstitution(
                                        [FindPackageShare('aist_bringup'),
                                         'launch', 'inc',
                                         'robotiq_devices.launch.py'])
    },
    'RobotiqGripper':
    {
        'gz_controllers_template':  PathJoinSubstitution(
                                        [FindPackageShare('aist_bringup'),
                                         'config', 'templates',
                                         'gripper_controllers.yaml']),
    },
    'DynamixelDevices':
    {
        'real_drivers_launch_file': PathJoinSubstitution(
                                        [FindPackageShare('aist_bringup'),
                                         'launch', 'inc',
                                         'dynamixel_devices.launch.py'])
    },
    'URioDevices':
    {
        'real_drivers_launch_file': PathJoinSubstitution(
                                        [FindPackageShare('aist_bringup'),
                                         'launch', 'inc',
                                         'ur_io_devices.launch.py'])
    },

    'PhoXiCamera':
    {
        'launch_file':        PathJoinSubstitution(
                                  [FindPackageShare('aist_phoxi_camera'),
                                   'launch', 'launch.py']),
        'key_of_id':          'id',
        'cloud_topic':        'pointcloud',
        'depth_topic':        'depth_map',
        'cinfo_topic':        'camera_info',
        'color_topic':        'texture',
        'normal_topic':       'normal_map',
        'gz_bridge_template': PathJoinSubstitution(
                                  [FindPackageShare('aist_bringup'),
                                   'config', 'templates',
                                   'rgbd_camera_bridge.yaml']),
    },
    'RealsenseCamera':
    {
        'launch_file':        PathJoinSubstitution(
                                  [FindPackageShare('realsense2_camera'),
                                   'launch', 'launch.py']),
        'key_of_id':          'serial_no',
        'cloud_topic':        'depth/color/points',
        'depth_topic':        'aligned_depth_to_color/image_raw',
        'cinfo_topic':        'aligned_depth_to_color/camera_info',
        'color_topic':        'color/image_raw',
        'gz_bridge_template': PathJoinSubstitution(
                                  [FindPackageShare('aist_bringup'),
                                   'config', 'templates',
                                   'rgbd_camera_bridge.yaml']),
    },
    'CodedLightRealsenseCamera':
    {
        'launch_file':        PathJoinSubstitution(
                                  [FindPackageShare('realsense2_camera'),
                                   'launch', 'launch.py']),
        'key_of_id':          'serial_no',
        'cloud_topic':        'depth/color/points',
        'depth_topic':        'aligned_depth_to_color/image_raw',
        'cinfo_topic':        'aligned_depth_to_color/camera_info',
        'color_topic':        'color/image_raw',
        'gz_bridge_template': PathJoinSubstitution(
                                  [FindPackageShare('aist_bringup'),
                                   'config', 'templates',
                                   'rgbd_camera_bridge.yaml']),
    },
    'USBCamera':
    {
        'launch_file':        PathJoinSubstitution(
                                  [FindPackageShare('aist_bringup'),
                                   'launch', 'inc', 'usb_cam.launch.py']),
        'key_of_id':          'video_device',
        'cinfo_topic':        'camera_info',
        'color_topic':        'image_raw',
        'gz_bridge_template': PathJoinSubstitution(
                                  [FindPackageShare('aist_bringup'),
                                   'config', 'templates',
                                   'area_camera_bridge.yaml']),
    },
}

def get_device_props(device_type):
    return DEVICE_PROPS.get(device_type)

def instantiate_file(context, template_file, instantiated_file, append=False):
    # We must extend lifetime of the ParameterFile object by keeping it
    # in a variable. Otherwise, the temporary file created by evaluating it
    # would be immediately erased by the destructor of ParameterFile.
    parameter_file = ParameterFile(template_file, allow_substs=True)
    if append:
        with open(parameter_file.evaluate(context)) as fin:
            content = fin.read()
            with open(instantiated_file, mode='a') as fout:
                fout.write(content)
    else:
        # Rename the created file to prevent from being erased.
        os.rename(parameter_file.evaluate(context), instantiated_file)
    parameter_file  # Extend lifetime
    return instantiated_file

def load_config(context):
    config_file = PathJoinSubstitution([FindPackageShare('aist_bringup'),
                                        'config',
                                        [LaunchConfiguration('config'),
                                         '.yaml']])
    with open(config_file.perform(context), 'r') as f:
        return yaml.safe_load(f)

def load_arm_config(arm_name):
    with open(filepath_from_url(
                  'package://aist_bringup/config/devices/arms.yaml')) as f:
        return yaml.safe_load(f)[arm_name]

def load_gripper_config(gripper_name):
    with open(filepath_from_url(
                  'package://aist_bringup/config/devices/grippers.yaml')) as f:
        return yaml.safe_load(f)[gripper_name]

def declare_launch_arguments(args):
    return [DeclareLaunchArgument(arg['name'],
                                  default_value=arg.get('default'),
                                  description=arg.get('description'),
                                  choices=arg.get('choices')) \
            for arg in args]
