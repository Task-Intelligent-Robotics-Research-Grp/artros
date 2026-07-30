# Software License Agreement (BSD License)
#
# Copyright (c) 2021, National Institute of Advanced Industrial Science and Technology (AIST)
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of National Institute of Advanced Industrial
#    Science and Technology (AIST) nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Toshio Ueshiba
#
import rclpy, sys, time, yaml, re
import numpy as np
import moveit_commander
import tf_transformations as tfs

from math                          import degrees, sqrt, pi
from rclpy.node                    import Node
from rclpy.duration                import Duration
from rclpy.time                    import Time
from rclpy.parameter               import Parameter
from rclpy.callback_groups         import MutuallyExclusiveCallbackGroup
from tf2_ros.buffer                import Buffer
from tf2_ros.transform_listener    import TransformListener
from std_msgs.msg                  import Header
from geometry_msgs.msg             import (PoseStamped, Pose, Point,
                                           Quaternion, PoseArray,
                                           Vector3, Vector3Stamped)
from moveit_msgs.msg               import (RobotTrajectory, PositionIKRequest,
                                           MoveItErrorCodes)
from moveit_msgs.srv               import GetPositionIK
from trajectory_msgs.msg           import JointTrajectoryPoint, JointTrajectory
from controller_manager_msgs.srv   import ListControllers, SwitchController
from action_msgs.msg               import GoalStatus
from aist_utility.fileio           import filepath_from_url
from aist_utility.geometry_msgs    import (transform_matrix, pose_matrix,
                                           pose_from_matrix)
from aist_tasks.pick_or_place_task import PickOrPlaceTask
from aist_collision_object_manager \
    .collision_object_manager      import CollisionObjectManager
from ddynamic_reconfigure2.utils   import declare_read_only_parameter
from .gripper_client               import create_gripper
from .camera_client                import CameraClient

#*********************************************************************
#  global functions                                                  *
#*********************************************************************
def get_grippers(config, name=''):
    if 'grippers' in config:
        grippers={}
        for gripper_name, gripper_config in config['grippers'].items():
            grippers |= get_grippers(gripper_config, gripper_name)
        return grippers
    return {} if name == '' else {name: config}

#*********************************************************************
#  class BaseRoutines                                                *
#*********************************************************************
class BaseRoutines(Node):
    """ Collection of basic routines for controlling arms, grippers
    and cameras.
    """
    ControllerTypes = (
        'joint_trajectory_controller/JointTrajectoryController',
        'forward_command_controller/ForwardCommandController',
        'cartesian_motion_controller/CartesianMotionController',
        'cartesian_force_controller/CartesianForceController',
        'cartesian_compliance_controller/CartesianComplianceController',
    )

    def __init__(self, name: str):
        """
        Args:
          name: Node name.
        """
        super().__init__(name)

        # Create TransformListener
        self._tf2_buffer   = Buffer()
        self._tf2_listener = TransformListener(self._tf2_buffer, self)

        time.sleep(1.0)        # Necessary for listener spinning up

        # MoveIt planning parameters
        self._eef_step        = self.declare_parameter('moveit_eef_step',
                                                       0.0005).value
        self._reference_frame = self.declare_parameter('reference_frame',
                                                       'world').value
        self._move_lin        = self.declare_parameter('move_lin',
                                                       True).value

        # MoveIt RobotCommander and MoveGroup
        moveit_commander.roscpp_initialize(sys.argv)
        self._cmd = moveit_commander.RobotCommander(
                        self.declare_parameter('robot_description',
                                               'robot_description').value)
        for group_name in self._cmd.get_group_names():
            group = self._cmd.get_group(group_name)
            group.set_pose_reference_frame(self.reference_frame)

        self.get_logger().info(
            'planning_frame: %s, reference_frame: %s, eef_step: %f'
            % (self.planning_frame, self.reference_frame, self.eef_step))

        # MoveIt GetPositionIK service client
        self._service_cbg = MutuallyExclusiveCallbackGroup()
        self._compute_ik = self.create_client(GetPositionIK, '/compute_ik',
                                              callback_group=self._service_cbg)

        # Hardware configuration
        with open(self.declare_parameter('config_file', name + '.yaml').value,
                  'r') as f:
            config = yaml.safe_load(f)

        # Controller_manager services for arms
        self._list_controllers_srvs \
            = {name: self.create_client(
                         ListControllers,
                         name + '/controller_manager/list_controllers',
                         callback_group=self._service_cbg)
               for name in config['arms']}

        self._switch_controller_srvs \
            = {name: self.create_client(
                         SwitchController,
                         name + '/controller_manager/switch_controller',
                         callback_group=self._service_cbg)
               for name in config['arms']}

        # Grippers
        self._grippers = {name: create_gripper(self, name, props['type'],
                                               props.get('client_args', {}))
                          for name, props in get_grippers(config).items()}
        self._default_gripper_names = {}
        self._active_grippers       = {}
        for name, props in config.get('arms', {}).items():
            self._default_gripper_names[name] = props['default_gripper']
            self.set_gripper(name, self.default_gripper_name(name))

        # Cameras
        self._cameras = {name: CameraClient.create(self, name, props['type'],
                                                   props.get('client_args',
                                                             {}))
                         for name, props in config.get('cameras', {}).items()}

        # Load setting parameters
        self.declare_parameter('setting_urls', [''])
        self.load_settings()

        # CollisionObjectManager wrapping MoveIt PlanningSceneInterface
        if 'initial_object_config' in self.settings:
            try:
                self._com = CollisionObjectManager(self)
            except Exception as e:
                self.get_logger().error(str(e))
                self._com = None
        else:
            self._com = None

        # Pick and place task
        self._pick_or_place  = PickOrPlaceTask(self)

        self.get_logger().info('BaseRoutines initialized.')

    def declare_parameter_with_type(self, name, type_, value):
        param = Parameter('tmp', type_=type_, value=value)
        return self.declare_parameter(name, param.get_parameter_value())

    @property
    def tf2_buffer(self) -> Buffer:
        """ TF2 buffer associated with this class.
        """
        return self._tf2_buffer

    @property
    def planning_frame(self) -> str:
        """ MoveIt planning frame.
        """
        return self._cmd.get_planning_frame()

    @property
    def reference_frame(self) -> str:
        """ MoveIt reference frame.
        """
        return self._reference_frame

    @property
    def eef_step(self) -> float:
        """ MoveIt end-effector step.
        """
        return self._eef_step

    @property
    def move_lin(self) -> bool:
        return self._move_lin

    @move_lin.setter
    def move_lin(self, enable):
        self._move_lin = enable

    @property
    def group_names(self) -> list[str]:
        """ Name list of MoveIt groups.
        """
        return self._cmd.get_group_names()

    @property
    def arm_names(self) -> list[str]:
        """ Name list of arms.
        """
        return self._list_controllers_srvs.keys()

    @property
    def gripper_names(self) -> list[str]:
        """ Name list of grippers.
        """
        return self._grippers.keys()

    @property
    def camera_names(self) -> list[str]:
        """ Name list of cameras.
        """
        return self._cameras.keys()

    @property
    def com(self) -> CollisionObjectManager:
        """ Client of collision object manager associated with this class.
        """
        return self._com

    @property
    def settings(self) -> dict:
        """ Settings for this class.
        The settings are loaded from files whose names are specified by
        the parameter 'setting_urls'.
        """
        return self._settings

    def load_settings(self) -> None:
        def recursive_merge(d1, d2):
            if type(d1) != dict or type(d2) != dict:
                return d2
            for k2, v2 in d2.items():
                d1[k2] = recursive_merge(d1.get(k2, {}), v2)
            return d1

        # Load setting parameters
        self._settings = {}
        for url in self.get_parameter('setting_urls') \
                       .get_parameter_value().string_array_value:
            try:
                with open(filepath_from_url(url), 'r') as f:
                    self._settings = recursive_merge(self._settings,
                                                     yaml.safe_load(f))
            except Exception as e:
                self.get_logger().error('failed to load setting file[%s]: %s'
                                        % (url, e))

    #
    # CLI(command line interface) stuffs
    #
    def print_help_messages(self):
        """ Print help messages for CLI(command-line interface).
        """
        print('=== General commands ===')
        print('  quit:        quit this program')
        print('  reload:      reload settings')
        print('  robot:       select robot')
        print('  ?|help:      print help messages')
        print('=== Arm commands ===')
        print('  X|Y|Z|R|P|W: select arm axis to be driven')
        print('  +|-:         move arm by 10(mm)/10(deg) along the current axis')
        print('  <numeric>:   move arm to the specified coordinate along the current axis')
        print('  home:        move arm to the home position')
        print('  back:        move arm to the back position')
        print('  named:       move arm to the pose specified by name')
        print('  frame:       move arm to the pose specified by frame')
        print('  clip:        make wrist angle within [-180, 180] deg.')
        print('  speed:       set speed')
        print('  stop:        stop arm immediately')
        print('  jvalues:     get current joint values')
        print('  switch:      switch controller')
        print('  toggle:      toggle motion control handle')
        print('  ftreset:     reset bias of ft-sensor')
        print('  lin:         enforce linear path')
        print('  LIN:         not enforce linear path')
        print('=== Gripper commands ===')
        print('  gripper:     assign gripper to current robot')
        print('  pregrasp:    pregrasp with the current gripper')
        print('  grasp:       grasp with the current gripper')
        print('  postgrasp:   postgrasp with the current gripper')
        print('  release:     release with the current gripper')
        print('  gpos:        set gripper position')
        print('  gvel:        set gripper velocity')
        print('  tighten:     tighten screw')
        print('  loosen:      loosen screw')
        print('  gcancel:     cancel gripper action')
        print('  pt:          pick tool')
        print('  PT:          place tool')
        print('=== Collision object commands ===')
        print('  I:  Initialize all collision objects')
        print('  i:  Show infomation on collision objects')
        print('  ci: Show infomation on child collision object of frame')
        print('  r:  Remove specified collision objects')

    def process_command(self, command: str, robot_name: str, axis: str,
                        speed: float=1.0) -> list[str, str, float]:
        """ Process interaction with user through CLI(command-line interface).

        Args:
          command:    Command input from user.
          robot_name: Robot name currently active.
          axis:       Axis name currently active.
          speed:      Current arm speed.

        Returns:
          Tuple of robot_name, axis and speed.
        """
        def _is_num(s):
            try:
                float(s)
            except ValueError:
                return False
            else:
                return True

        def _get_offset():
            offset = []
            for s in input('  offset? ').split():
                if _is_num(s):
                    offset.append(float(s))
                else:
                    return None
            return offset

        if command == 'quit':
            self.go_to_named_pose(robot_name, 'home')  # Reset pose
            rclpy.shutdown()
        elif command == 'reload':
            self.load_settings()
        elif command == 'robot':
            print('  current: %s' % robot_name)
            new_robot_name = input('  robot name? ')
            if new_robot_name in self.arm_names:
                robot_name = new_robot_name
            else:
                self.get_logger().error('Unknown robot name[%s]'
                                        % new_robot_name)
        elif command == '?' or command == 'help':
            self.print_help_messages()
            print('')

        # Arm stuffs
        elif command == 'X':
            axis = 'X'
        elif command == 'Y':
            axis = 'Y'
        elif command == 'Z':
            axis = 'Z'
        elif command == 'R':
            axis = 'Roll'
        elif command == 'P':
            axis = 'Pitch'
        elif command == 'W':
            axis = 'Yaw'
        elif command == '+':
            offset = [0, 0, 0, 0, 0, 0]
            if axis == 'X':
                offset[0] = 0.01
            elif axis == 'Y':
                offset[1] = 0.01
            elif axis == 'Z':
                offset[2] = 0.01
            elif axis == 'Roll':
                offset[3] = 10.0
            elif axis == 'Pitch':
                offset[4] = 10.0
            else:
                offset[5] = 10.0
            self.move_relative(robot_name, offset, speed)
        elif command == '-':
            offset = [0, 0, 0, 0, 0, 0]
            if axis == 'X':
                offset[0] = -0.01
            elif axis == 'Y':
                offset[1] = -0.01
            elif axis == 'Z':
                offset[2] = -0.01
            elif axis == 'Roll':
                offset[3] = -10.0
            elif axis == 'Pitch':
                offset[4] = -10.0
            else:
                offset[5] = -10.0
            self.move_relative(robot_name, offset, speed)
        elif _is_num(command):
            xyzrpy = self.xyzrpy_from_pose(self.get_current_pose(robot_name))
            if axis == 'X':
                xyzrpy[0] = float(command)
            elif axis == 'Y':
                xyzrpy[1] = float(command)
            elif axis == 'Z':
                xyzrpy[2] = float(command)
            elif axis == 'Roll':
                xyzrpy[3] = float(command)
            elif axis == 'Pitch':
                xyzrpy[4] = float(command)
            else:
                xyzrpy[5] = float(command)
            self.go_to_pose_goal(robot_name,
                                 self.pose_from_xyzrpy(xyzrpy), speed=speed)
        elif command == 'home':
            self.go_to_named_pose(robot_name, 'home')
        elif command == 'back':
            self.go_to_named_pose(robot_name, 'back')
        elif command == 'named':
            pose_name = input('  pose name? ')
            try:
                self.go_to_named_pose(robot_name, pose_name, speed=speed)
            except rclpy.ROSException as e:
                self.get_logger().error('Failed to go to pose[%s]: %s'
                                        % (pose_name, e))
        elif command == 'frame':
            frame    = input('  frame? ')
            offset   = _get_offset()
            eef_link = input('  eef_link? ')
            try:
                self.go_to_frame(robot_name, frame, offset, speed=speed,
                                 end_effector_link=eef_link)
            except Exception as e:
                self.get_logger().error('Failed to go to frame[%s]: %s'
                                        % (frame, e))
        elif command == 'clip':
            self.clip_wrist_joint_value(robot_name)
        elif command == 'speed':
           speed = float(input('  speed value? '))
        elif command == 'stop':
            self.stop(robot_name)
        elif command == 'jvalues':
            print(self.get_current_joint_values(robot_name))
        elif command == 'switch':
            controllers = self.list_controllers(robot_name)
            print('  available controllers:')
            for n, controller in enumerate(controllers):
                if controller.state == 'active':
                    print('   *%2d. %s' % (n, controller.name))
                else:
                    print('    %2d. %s' % (n, controller.name))
            try:
                self.switch_controller(
                    robot_name,
                    controllers[int(input('  controller #? '))].name)
            except:
                self.get_logger().error('Invalid index!')
        elif command == 'toggle':
            self.toggle_control_handle(robot_name)
        elif command == 'ftreset':
            self.ftsensor_reset_bias(robot_name)
        elif command == 'lin':
            self.move_lin = True
        elif command == 'LIN':
            self.move_lin = False

        # Gripper stuffs
        elif command == 'gripper':
            print('  current: %s' % self.gripper(robot_name).name)
            try:
                self.set_gripper(robot_name, input('  gripper name? '))
            except Exception as e:
                self.get_logger().error('Failed to set gripper: %s' % e)
        elif command == 'pregrasp':
            self.pregrasp(robot_name)
        elif command == 'grasp':
            self.grasp(robot_name)
        elif command == 'postgrasp':
            self.postgrasp(robot_name)
        elif command == 'release':
            self.release(robot_name)
        elif command == 'gpos':
            position = float(input('  position? '))
            self.set_gripper_position(robot_name, position)
        elif command == 'gvel':
            velocity = float(input('  velocity? '))
            self.set_gripper_velocity(robot_name, velocity)
        elif command == 'tighten':
            self.tighten(robot_name)
        elif command == 'loosen':
            self.loosen(robot_name)
        elif command == 'gcancel':
            self.gripper_cancel(robot_name)
        elif command == 'pt':
            tool_name = input('  tool name? ')
            self.pick_tool(robot_name, tool_name)
        elif command == 'PT':
            self.place_tool(robot_name)

        # Collision objects stuffs
        elif command == 'I':
            self.initialize_collision_objects()
        elif command == 'i':
            object_id = input('  object ID? ')
            info = self.com.get_object_info(object_id)
            if info is not None:
                self.print_object_info(info)
        elif command == 'ci':
            frame_id = input('  parent frame? ')
            info_list = self.com.get_attached_child_objects_info(frame_id)
            for info in info_list:
                self.print_object_info(info)
                print('----------------')
        elif command == 'r':
            object_id   = input('  object_id? ')
            attach_link = input('  attach_link? ') if object_id == '' else ''
            self.com.remove_object(object_id, attach_link)

        else:
            print('  unknown command! [%s]' % command)
        return robot_name, axis, speed

    #
    # Joint motion stuffs
    #
    def get_joint_names(self, robot_name):
        return self._cmd.get_group(robot_name).get_active_joints()

    def get_current_joint_values(self, robot_name):
        return self._cmd.get_group(robot_name).get_current_joint_values()

    def get_named_joint_values(self, robot_name, named_pose):
        target_values = self._cmd.get_group(robot_name)\
                                 .get_named_target_values(named_pose)
        return [target_values[joint_name]
                for joint_name in self.get_joint_names(robot_name)]

    def remember_joint_values(self, robot_name, name, joint_values=None):
        self._cmd.get_group(robot_name).remember_joint_values(name,
                                                              joint_values)

    def go_to_named_pose(self, robot_name, named_pose, speed=1.0, accel=1.0):
        group = self._cmd.get_group(robot_name)
        try:
            group.set_named_target(named_pose)
        except moveit_commander.exception.MoveItCommanderException as e:
            self.get_logger().error('AistBaseRoutines.go_to_named_pose(): %s'
                                    % e)
            return False
        return self._go(group, speed, accel)

    def go_to_joint_value_target(self, robot_name, joint_values,
                                 speed=1.0, accel=1.0):
        group = self._cmd.get_group(robot_name)
        group.set_joint_value_target(joint_values)
        return self._go(group, speed, accel)

    def clip_wrist_joint_value(self, robot_name, speed=1.0, accel=1.0):
        joint_values = self.get_current_joint_values(robot_name)
        if joint_values[-1] >= -pi:
            if joint_values[-1] <= pi:
                return True
            joint_values[-1] -= 2*pi
        else:
            joint_values[-1] += 2*pi
        return self.go_to_joint_value_target(robot_name, joint_values,
                                             speed, accel)

    def go_directly_to_joint_value_target(self, robot_name,
                                          joint_values, duration):
        joint_trajectory \
            = JointTrajectory(
                joint_names=self.joint_names,
                points=[
                    JointTrajectoryPoint(
                        positions=self.get_current_joint_values(robot_name),
                        time_from_start=Duration(seconds=0)),
                    JointTrajectoryPoint(positions=joint_values,
                                         time_from_start=duration)])
        return self.execute_path(robot_name,
                                 RobotTrajectory(
                                     joint_trajectory=joint_trajectory))

    def _go(self, group, speed=1.0, accel=1.0):
        group.set_max_velocity_scaling_factor(np.clip(speed, 0.0, 1.0))
        group.set_max_acceleration_scaling_factor(np.clip(accel, 0.0, 1.0))
        success = group.go(wait=True)
        if not success:
            self.get_logger().error('Failed to go to target.')
        group.clear_pose_targets()
        return success

    #
    # Cartesian motion stuffs
    #
    def get_current_pose(self, robot_name: str) -> PoseStamped:
        """ Get current pose of the specified robot.

        Args:
          robot_name: Robot name.

        Returns:
          PoseStamped: Current pose of the robot.
        """
        group    = self._cmd.get_group(robot_name)
        eef_link = self.gripper(robot_name).tip_link
        if eef_link in self._cmd.get_link_names():
            return group.get_current_pose()

        default_eef_link = self.default_gripper(robot_name).tip_link
        pose = group.get_current_pose(default_eef_link)
        tfm = self.lookup_transform(default_eef_link, eef_link, Time(),
                                    Duration(seconds=10)).transform
        T = tfs.translation_matrix((pose.pose.position.x,
                                    pose.pose.position.y,
                                    pose.pose.position.z)) \
          @ tfs.quaternion_matrix((pose.pose.orientation.x,
                                   pose.pose.orientation.y,
                                   pose.pose.orientation.z,
                                   pose.pose.orientation.w)) \
          @ tfs.translation_matrix((tfm.translation.x,
                                    tfm.translation.y,
                                    tfm.translation.z)) \
          @ tfs.quaternion_matrix((tfm.rotation.x, tfm.rotation.y,
                                   tfm.rotation.z, tfm.rotation.w))
        t = tfs.translation_from_matrix(T)
        q = tfs.quaternion_from_matrix(T)
        pose.pose = Pose(position=Point(x=t[0], y=t[1], z=t[2]),
                         orientation=Quaternion(x=q[0], y=q[1],
                                                z=q[2], w=q[3]))
        return pose

    def move_relative(self, robot_name: str, offset,
                      speed: float=1.0, accel: float=1.0,
                      end_effector_link: str='') -> bool:
        """ Move the end-effector from current pose by given offset values.
        Offset is specified by a tuple with three, six of seven float values.

        Args:
          robot_name: Robot name.
          offset:
          speed: Upper-limit ratio relative to the maximum speed.
          accel: Upper-limit ratio relative to the maximum acceleration.
          end_effector_link: Link name of the end-effector. If empty,
            use tip_link of the gripper currently attached to the robot.

        Returns:
          bool: `True` iff success.
        """
        return self.go_to_pose_goal(robot_name,
                                    self.get_current_pose(robot_name),
                                    offset, speed, accel, end_effector_link)

    def go_to_frame(self, robot_name, target_frame, offset=(),
                    speed=1.0, accel=1.0, end_effector_link=''):
        return self.go_to_pose_goal(robot_name,
                                    PoseStamped(
                                        header=Header(frame_id=target_frame),
                                        pose=Pose(
                                            position=Point(
                                                x=0.0, y=0.0, z=0.0),
                                            orientation=Quaternion(
                                                x=0.0, y=0.0, z=0.0, w=1.0))),
                                    offset, speed, accel, end_effector_link)

    def go_to_pose_goal(self, robot_name, target_pose, offset=(),
                        speed=1.0, accel=1.0, end_effector_link=''):
        return self.go_along_poses(robot_name,
                                   PoseArray(header=target_pose.header,
                                             poses=[target_pose.pose]),
                                   offset, speed, accel, end_effector_link)

    def go_along_poses(self, robot_name, poses, offset=(),
                       speed=1.0, accel=1.0, end_effector_link=''):
        path = self.create_path(robot_name, poses, offset,
                                speed, accel, end_effector_link)
        if path is None:
            return False
        # group = self._cmd.get_group(robot_name)
        # return self.execute_path(robot_name,
        #                          group.retime_trajectory(
        #                              self._cmd.get_current_state(), path,
        #                              velocity_scaling_factor=speed,
        #                              acceleration_scaling_factor=accel))
        return self.execute_path(robot_name, path)

    def execute_path(self, robot_name, path):
        success = self._cmd.get_group(robot_name).execute(path, wait=True)
        if not success:
            self.get_logger().error('Failed to execute path.')
        self.stop(robot_name)
        return success

    def create_path(self, robot_name, poses, offset=(),
                    speed=1.0, accel=1.0, end_effector_link=''):
        if end_effector_link == '':
            end_effector_link = self.gripper(robot_name).tip_link
        group = self._cmd.get_group(robot_name)
        group.set_end_effector_link(end_effector_link)

        group.set_max_velocity_scaling_factor(np.clip(speed, 0.0, 1.0))
        group.set_max_acceleration_scaling_factor(np.clip(accel, 0.0, 1.0))
        transformed_poses = self.transform_poses_to_target_frame(poses, offset)

        if not self.move_lin:
            group.set_start_state_to_current_state()
            group.set_pose_target(transformed_poses.poses[-1],
                                  end_effector_link)
            success, path, planning_time, error_code = group.plan()
            if not success:
                self.get_logger().error('Failed to compute non-linear path: planning_time=%f, error=%s@%s[%d]]'
                                        % (planning_time, error_code.message,
                                           error_code.source, error_code.val))
                return None
            return path[0]

        try:
            path, fraction = group.compute_cartesian_path(
                                 transformed_poses.poses, self._eef_step, 0.0)
        except Exception as e:
            self.get_logger().error(e)
            return None

        if fraction < 0.995:
            self.get_logger().error('Computed only %3.1f%% of cartesian path.'
                                    % (100.0*fraction))
            return None
        self.get_logger().info('Computed %3.1f%% of cartesian path.'
                               % (100.0*fraction))
        return path

    def create_timed_path(self, robot_name, poses, times_from_start):
        robot_state = self._cmd.get_current_state()
        robot_state.joint_state.header.stamp = poses.header.stamp
        req = PositionIKRequest(group_name=robot_name,
                                robot_state=robot_state)
        joint_trajectory = JointTrajectory(req.robot_state.joint_state.header,
                                           req.robot_state.joint_state.name,
                                           [])
        transformed_poses = self.transform_poses_to_target_frame(poses)
        for pose, time_from_start in zip(transformed_poses.poses,
                                         times_from_start):
            req.pose_stamped = PoseStamped(transformed_poses.header, pose)
            res = self._compute_ik(req)
            if res.error_code.val != MoveItErrorCodes.SUCCESS:
                self.get_logger().error('Failed to solve IK[%d]'
                                        % res.error_code.val)
                return None
            joint_state = res.solution.joint_state
            point = JointTrajectoryPoint(positions=joint_state.position,
                                         velocities=joint_state.velocity,
                                         effort=joint_state.effort)
            point.time_from_start = time_from_start
            joint_trajectory.points.append(point)
        return RobotTrajectory(joint_trajectory=joint_trajectory)

    def stop(self, robot_name):
        group = self._cmd.get_group(robot_name)
        group.stop()
        group.clear_pose_targets()

    #
    # Controller stuffs
    #
    def list_controllers(self, robot_name: str):
        """ List of controllers associated with the robot.

        Args:
          robot_name: Robot name.

        Returns:
          List of controllers.
        """
        return list(filter(lambda x: x.type in BaseRoutines.ControllerTypes,
                           self._list_controllers_srvs[robot_name].call(
                               ListControllers.Request()).controller))

    def current_controller(self, robot_name: str):
        """ Current active controller associated with the robot.

        Args:
          robot_name: Robot name.

        Returns:
          Active contoller, if exists. `None`, if no active controllers.
        """
        for controller in self.list_controllers(robot_name):
            if controller.state == 'active':
                return controller
        return None

    def switch_controller(self, robot_name: str, controller_name: str) -> bool:
        """ Current active controller associated with the robot.

        Args:
          robot_name:      Robot name.
          controller_name: Name of the controller to be switched to.

        Returns:
          bool: `True`, if success. `False`, if failure.
        """
        for controller in self.list_controllers(robot_name):
            if controller.name == controller_name:
                if controller.state == 'active':
                    self.get_logger().warn('Already active[%s]'
                                           % controller_name)
                    return True
                elif controller.state == 'unconfigured' or \
                     controller.state == 'inactive':
                    # if controller.type == 'cartesian_force_controller/CartesianForceController' or \
                    #    controller.type == 'cartesian_compliance_controller/CartesianComplianceController':
                    #     self.ftsensor_reset_bias()
                    current_controller = self.current_controller(robot_name)
                    req = SwitchController.Request()
                    req.activate_controllers   = [controller_name]
                    req.deactivate_controllers = [] if not current_controller \
                                                 else [current_controller.name]
                    req.strictness             = SwitchController.Request.STRICT
                    req.activate_asap          = True
                    req.timeout                = Duration(seconds=1).to_msg()
                    res = self._switch_controller_srvs[robot_name].call(req)
                    time.sleep(0.5)
                    if res.ok:
                        self.get_logger().info(
                            'Succesfully switched to controller[%s]'
                            % controller_name)
                    else:
                        self.get_logger().error(
                            'Failed to switch to controller[%s]'
                            % controller_name)
                    return res.ok
                else:
                    self.get_logger().warn(
                        "Controller state is '%', returning True."
                        % controller.state)
                    return True
        self.get_logger().error('Specified controller[%s] not found'
                                % controller_name)
        return False

    def toggle_control_handle(self, robot_name: str) -> bool:
        """ Activate/deactivate motion control handle.

        Args:
          robot_name: Robot name.

        Returns:
          bool: `True`, if successfully switched. `False`, if failure.
        """
        controller_name = robot_name + '_motion_control_handle'
        for controller in self.list_controllers(robot_name):
            if controller.name == controller_name:
                req = SwitchController.Request()
                if controller.state == 'active':
                    req.activate_controllers    = []
                    req.deactivate_controllers  = [controller_name]
                    message = 'deactivated ' + controller_name
                    self.switch_controller(robot_name,
                                           robot_name +
                                           '_scaled_pos_joint_traj_controller')
                else:
                    self.switch_controller(robot_name,
                                           robot_name +
                                           '_cartesian_compliance_controller')
                    req.activate_controllers = [controller_name]
                    req.deactivate_controllers  = []
                    message = 'activated ' + controller_name
                req.strictness        = SwitchControllerRequest.STRICT
                req.start_asap        = True
                req.timeout           = Duration(seconds=1).to_msg()
                res = self._switch_controller.call(req)
                time.sleep(0.5)
                if res.ok:
                    self.get_logger().info('Succesfully %s' % message)
                else:
                    self.get_logger().error('Failed to %s' % message)
                return res.ok
        return False

    #
    # Gripper stuffs
    #
    def _create_gripper(self, name, type_name, props):
        gripper_client_class = globals().get(type_name)
        if gripper_client_class is None:
            raise RuntimeError('unknown type[%s] of the gripper[%s]'
                               % (type_name, name))
        return gripper_client_class(self, name, **props)

    def default_gripper_name(self, robot_name):
        return self._default_gripper_names[robot_name]

    def default_gripper(self, robot_name):
        return self._grippers[self.default_gripper_name(robot_name)]

    def set_gripper(self, robot_name, gripper_name):
        gripper = self._grippers.get(gripper_name)
        if gripper is None:
            raise RuntimeError('unknown gripper[%s]' % gripper_name)
        self._active_grippers[robot_name] = gripper

    def gripper(self, robot_name):
        return self._active_grippers[robot_name]

    def set_gripper_parameters(self, robot_name, parameters):
        self.gripper(robot_name).set_parameters(parameters)

    def gripper_parameters(self, robot_name):
        return self.gripper(robot_name).parameters

    def pregrasp(self, robot_name):
        return self.gripper(robot_name).pregrasp()

    def grasp(self, robot_name):
        return self.gripper(robot_name).grasp()

    def postgrasp(self, robot_name):
        return self.gripper(robot_name).postgrasp()

    def release(self, robot_name):
        return self.gripper(robot_name).release()

    def set_gripper_position(self, robot_name, position):
        return self.gripper(robot_name).move(position)

    def set_gripper_velocity(self, robot_name, velocity):
        return self.gripper(robot_name).set_velocity(velocity)

    def tighten(self, robot_name):
        self.gripper(robot_name).tighten()

    def loosen(self, robot_name):
        self.gripper(robot_name).loosen()

    def gripper_cancel(self, robot_name):
        self.gripper(robot_name).cancel_goal()

    #
    # Camera stuffs
    #
    def camera(self, camera_name):
        return self._cameras[camera_name]

    def continuous_shot(self, camera_name, enable):
        return self.camera(camera_name).continuous_shot(enable)

    def trigger_frame(self, camera_name):
        return self.camera(camera_name).trigger_frame()

    #
    # Pick and place action stuffs
    #
    def pick(self, robot_name, part_id, target_pose, *, timeout_sec=None):
        picking_parameters = self.settings.get('picking_parameters', {})
        params             = picking_parameters.get(part_id)
        #self.get_logger().info('### [%s] %s' % (part_id, picking_params))
        if params is None:
            params = picking_parameters[
                         self.com.get_object_info(part_id).object_type]
        self.set_gripper_parameters(robot_name,
                                    params.get('gripper_parameters', {}))
        return self._pick_or_place.send_goal(robot_name, True, target_pose,
                                             params['pick_offset'],
                                             params['approach_offset'],
                                             params['departure_offset'],
                                             params['speed_fast'],
                                             params['speed_slow'],
                                             end_effector_link='',
                                             timeout_sec=timeout_sec)

    def place(self, robot_name, part_id, target_pose,
              *, subframe='base_link', timeout_sec=None):
        picking_parameters = self.settings.get('picking_parameters', {})
        picking_params     = picking_parameters.get(part_id)
        if picking_params is None:
            picking_params = picking_parameters[
                                 self.com.get_object_info(part_id).object_type]
        placing_parameters = self.settings.get('placing_parameters', {})
        placing_params     = placing_parameters.get(
                                 target_pose.header.frame_id, picking_params)
        if not placing_params.get('place_offset'):
            placing_params = placing_parameters['default']
        self.set_gripper_parameters(robot_name,
                                    picking_params.get('gripper_parameters',
                                                       {}))
        eef_link = part_id + '/' + subframe if part_id != '' else ''
        return self._pick_or_place.send_goal(robot_name, False, target_pose,
                                             placing_params['place_offset'],
                                             placing_params['approach_offset'],
                                             placing_params['departure_offset'],
                                             picking_params['speed_fast'],
                                             picking_params['speed_slow'],
                                             end_effector_link=eef_link,
                                             timeout_sec=timeout_sec)

    def pick_at_frame(self, robot_name, part_id, target_frame,
                      *, offset=(), timeout_sec=None):
        return self.pick(robot_name, part_id,
                         self.pose_from_xyzrpy(offset, target_frame),
                         timeout_sec=timeout_sec)

    def place_at_frame(self, robot_name, part_id, target_frame,
                       *, offset=(), subframe='base_link', timeout_sec=None):
        return self.place(robot_name, part_id,
                          self.pose_from_xyzrpy(offset, target_frame),
                          subframe=subframe, timeout_sec=timeout_sec)

    def pick_tool(self, robot_name, tool_name, *, timeout_sec=None):
        if tool_name not in self._grippers:
            self.get_logger().error('Unknown tool name[%s]' % tool_name)
            return (GoalStatus.STATUS_ABORTED, None)
        if self.gripper(robot_name).name == tool_name:
            return (GoalStatus.STATUS_SUCCEEDED, None)
        if self.gripper(robot_name).name != \
           self.default_gripper_name(robot_name):
            self.place_tool(robot_name)
        status, result = self.pick_at_frame(robot_name, tool_name,
                                            tool_name + '/base_link',
                                            timeout_sec=timeout_sec)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.set_gripper(robot_name, tool_name)
            #self.ftsensor_reset_bias(robot_name)
        return (status, result)

    def place_tool(self, robot_name, *, timeout_sec=None):
        tool_name            = self.gripper(robot_name).name
        default_gripper_name = self.default_gripper_name(robot_name)
        if tool_name == default_gripper_name:
            return (GoalStatus.STATUS_SUCCEEDED, None)
        self.set_gripper(robot_name, default_gripper_name)
        return self.place_at_frame(robot_name, tool_name,
                                   tool_name + '_holder_link',
                                   timeout_sec=timeout_sec)

    def pick_or_place_wait(self, *, target_stage=None, timeout_sec=None):
        return self._pick_or_place.wait(target_stage=target_stage,
                                        timeout_sec=timeout_sec)

    def pick_or_place_cancel_goal(self):
        self._pick_or_place.cancel_goal()

    #
    # Utility functions
    #
    def initialize_collision_objects(self):
        self.com.remove_object()
        for object_type, config in self.settings.get('initial_object_config',
                                                     {}).items():
            self.com.create_object(object_type,
                                   self.pose_from_xyzrpy(
                                       config.get('offset', ()),
                                       config['parent_link']),
                                   config.get('subframe', 'base_link'))
            self.com.allow_collision(object_type, config['parent_link'])
            if config.get('attach', False):
                self.com.attach_object(object_type, config['parent_link'])

    def print_object_info(self, info):
        print('    object_id:   %s\n    type:        %s\n    parent_link: %s\n    attach_link: %s\n    touch_links: %s\n    acm_allowed: %s\n    pose:        %s@%s'
              % (info.object_id, info.object_type, info.parent_link,
                 info.attach_link, info.touch_links, info.acm_allowed,
                 self.format_pose(info.pose, info.pose.header.frame_id),
                 info.pose.header.frame_id))

    def lookup_transform(self, target_frame, source_frame,
                         time=Time(), timeout=Duration()):
        try:
            return self._tf2_buffer.lookup_transform(target_frame,
                                                     source_frame,
                                                     time, timeout)
        except Exception as e:
            self.get_logger().error('BaseRoutines.lookup_transform(): %s' % e)
            raise e

    def transform_points_to_target_frame(self, header, points,
                                         target_frame=''):
        if target_frame == '':
            target_frame = self._reference_frame

        T = transform_matrix(self.lookup_transform(
                                 target_frame, header.frame_id,
                                 header.stamp, Duration(seconds=10)).transform)
        transformed_points = []
        for point in points:
            p = T @ (point.x, point.y, point.z, 1.0)
            transformed_points.append(Point(x=p[0], y=p[1], z=p[2]))
        return transformed_points

    def transform_pose_to_target_frame(self, pose, offset=(), target_frame=''):
        poses = self.transform_poses_to_target_frame(
                    PoseArray(header=pose.header, poses=[pose.pose]),
                    offset, target_frame)
        return PoseStamped(header=poses.header, pose=poses.poses[0])

    def transform_poses_to_target_frame(self, poses,
                                        offset=(), target_frame=''):
        if target_frame == '':
            target_frame = self.reference_frame

        T = transform_matrix(self.lookup_transform(
                                 target_frame, poses.header.frame_id,
                                 poses.header.stamp,
                                 Duration(seconds=10)).transform)
        S = tfs.translation_matrix(self._position_from_offset(offset[0:3])) \
          @ tfs.quaternion_matrix(self._orientation_from_offset(offset[3:]))
        return PoseArray(header=Header(frame_id=target_frame,
                                       stamp=poses.header.stamp),
                         poses=[pose_from_matrix(T @ pose_matrix(pose) @ S)
                                for pose in poses.poses])

    def correct_orientation(self, pose):
        poses = self.correct_orientations(PoseArray(header=pose.header,
                                                    poses=[pose.pose]))
        return PoseStamped(header=poses.header, pose=poses.poses[0])

    def correct_orientations(self, poses):
        up = self._tf2_buffer.transformVector3(
                 poses.header.frame_id,
                 Vector3Stamped(Header(stamp=poses.header.stamp,
                                       frame_id=self.reference_frame),
                                Vector3(x=0, y=0, z=1)))
        return PoseArray(header=poses.header,
                         poses=[Pose(position=pose.position,
                                     orientation=self._correct_orientation(
                                         pose.orientation, up.vector))
                                for pose in poses.poses])

    def pose_from_xyzrpy(self, xyzrpy, frame_id=''):
        if frame_id == '':
            frame_id = self.reference_frame
        t = self._position_from_offset(xyzrpy[0:3])
        q = self._orientation_from_offset(xyzrpy[3:])
        return PoseStamped(header=Header(frame_id=frame_id),
                           pose=Pose(position=Point(x=t[0], y=t[1], z=t[2]),
                                     orientation=Quaternion(x=q[0], y=q[1],
                                                            z=q[2], w=q[3])))

    def xyzrpy_from_pose(self, pose, target_frame=''):
        transformed_pose = self.transform_pose_to_target_frame(
                               pose, target_frame=target_frame).pose
        rpy = tfs.euler_from_quaternion((transformed_pose.orientation.x,
                                         transformed_pose.orientation.y,
                                         transformed_pose.orientation.z,
                                         transformed_pose.orientation.w))
        return [transformed_pose.position.x,
                transformed_pose.position.y,
                transformed_pose.position.z,
                degrees(rpy[0]), degrees(rpy[1]), degrees(rpy[2])]

    def format_pose(self, target_pose, target_frame=''):
        return '[{:.4f}, {:.4f}, {:.4f}; {:.2f}, {:.2f}. {:.2f}]'.format(
            *self.xyzrpy_from_pose(target_pose, target_frame))

    #
    # Private functions
    #
    def _position_from_offset(self, offset):
        return np.array((0.0, 0.0, 0.0) if len(offset) < 3 else offset[0:3])

    def _orientation_from_offset(self, offset):
        return np.array((0.0, 0.0, 0.0, 1.0)) if len(offset) < 3 else \
               tfs.quaternion_from_euler(
                   *np.radians(offset[0:3])) if len(offset) == 3 else \
               np.array(offset[0:4])

    def _correct_orientation(self, orientation, up):
        q     = (orientation.x, orientation.y, orientation.z, orientation.w)
        r     = tfs.quaternion_matrix(q)[0:3, 2]  # current up vector
        n     = (up.x, up.y, up.z)                # desired up vector
        a     = np.cross(r, n)                    # rotation axis
        dq    = np.empty(4)
        dq[3] = sqrt(0.5 + 0.5*np.dot(r, n))
        if abs(dq[3]) < 1e-7:                     # n == -r ?
            dq[0:3] = (sqrt(0.5), sqrt(0.5), 0.0) # swap x and y, then flip z
        else:
            dq[0:3] = (0.5/dq[3])*a
        q_corrected = tfs.quaternion_multiply(q, dq)
        return Quaternion(x=q_corrected.x, y=q_corrected.y,
                          z=q_corrected.z, w=q_corrected.w)
