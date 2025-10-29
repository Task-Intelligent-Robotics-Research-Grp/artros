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
import rclpy, sys, time, yaml
import numpy as np
import moveit_commander
import tf_transformations as tfs

from math                                 import degrees, sqrt, pi
from rclpy.node                           import Node
from rclpy.duration                       import Duration
from rclpy.time                           import Time
from rclpy.parameter                      import Parameter
from tf2_ros.buffer                       import Buffer
from tf2_ros.transform_listener           import TransformListener
from rcl_interfaces.msg                   import ParameterDescriptor
from std_msgs.msg                         import Header
from geometry_msgs.msg                    import (PoseStamped, Pose, Point,
                                                  Quaternion, PoseArray,
                                                  Vector3, Vector3Stamped)
from moveit_msgs.msg                      import (RobotTrajectory,
                                                  PositionIKRequest,
                                                  MoveItErrorCodes)
from moveit_msgs.srv                      import GetPositionIK
from trajectory_msgs.msg                  import (JointTrajectoryPoint,
                                                  JointTrajectory)
from aist_routines.gripper_client         import GripperClient
from aist_routines.camera_client          import CameraClient
#from aist_routines.MarkerPublisher    import MarkerPublisher
from aist_utility.fileio                  import filepath_from_url
from aist_collision_object_manager.client import CollisionObjectManagerClient
from ddynamic_reconfigure2.utils          import declare_read_only_parameter

######################################################################
#  global functions                                                  #
######################################################################
def get_grippers(config, name=''):
    if 'grippers' in config:
        grippers={}
        for gripper_name, gripper_config in config['grippers'].items():
            grippers |= get_grippers(gripper_config, gripper_name)
        return grippers
    return {name: config}

def paramtuples(d):
    fields = set()
    for params in d.values():
        for field in params.keys():
            fields.add(field)
    ParamTuple = collections.namedtuple('ParamTuple', ' '.join(fields))

    params = {}
    for key, param in d.items():
        params[key] = ParamTuple(**param)
    return params

######################################################################
#  class AISTBaseRoutines                                            #
######################################################################
class AISTBaseRoutines(Node):
    def __init__(self, name):
        super().__init__(name)

        # Create TransformListener
        self._tf2_buffer   = Buffer()
        self._tf2_listener = TransformListener(self._tf2_buffer, self)

        time.sleep(1.0)        # Necessary for listner spinning up

        # MoveIt planning parameters
        self._eef_step = self.declare_parameter('moveit_eef_step',
                                                0.0005).value
        self._reference_frame = self.declare_parameter('reference_frame',
                                                       'world').value

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
        self._compute_ik = self.create_client(GetPositionIK, '/compute_ik')

        # Hardware configuration
        with open(self.declare_parameter('config_file', name + '.yaml').value,
                  'r') as f:
            config = yaml.safe_load(f)

        # Grippers
        self._grippers = {name: GripperClient.create(self, name, props['type'],
                                                     props.get('client_args',
                                                               {}))
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
        self._settings = {}
        for url in self.declare_parameter('setting_urls', [''], ParameterDescriptor(type=Parameter.Type.STRING_ARRAY)).value:
            with open(filepath_from_url(url), 'r') as f:
                self._settings |= yaml.safe_load(f)

        # CollisionObjectManager wrapping MoveIt PlanningSceneInterface
        if 'initial_object_config' in self.settings:
            try:
                self._com = CollisionObjectManagerClient(self)
            except Exception as e:
                self.get_logger().error(str(e))
                self._com = None
        else:
            self._com = None

        # Pick and place action
        # if rospy.has_param('~picking_parameters'):
        #     self._picking_params = rospy.get_param('~picking_parameters')
        #     self._placing_params = rospy.get_param('~placing_parameters', {})
        #     self._pick_or_place  = PickOrPlace(self)
        # else:
        #     self._pick_or_place = None

        # Spiral motion action
        # self._spiral_motion = SpiralMotion(self)

        # Marker publisher
        # self._markerPublisher = MarkerPublisher()

        self.get_logger().info('AISTBaseRoutines initialized.')

    def __enter__(self):
        return self

    # def __exit__(self, exception_type, exception_value, traceback):
    #     if self._pick_or_place:
    #         self._pick_or_place.shutdown()
    #     rospy.signal_shutdown('AISTBaseRoutines() completed.')
    #     return False  # Do not forward exceptions

    @property
    def tf2_buffer(self):
        return self._tf2_buffer

    @property
    def planning_frame(self):
        return self._cmd.get_planning_frame()

    @property
    def reference_frame(self):
        return self._reference_frame

    @property
    def eef_step(self):
        return self._eef_step

    @property
    def group_names(self):
        return self._cmd.get_group_names()

    @property
    def com(self):
        return self._com

    @property
    def settings(self):
        return self._settings

    # Interactive stuffs
    def print_help_messages(self):
        print('=== General commands ===')
        print('  quit:        quit this program')
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
        print('  sm:          spiral motion')
        print('  SM:          cancel spiral motion')
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
        print('  gcancel:     cancel tighten/loosen action')

    def interactive(self, key, robot_name, axis, speed=1.0):
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

        if key == 'quit':
            self.go_to_named_pose(robot_name, 'home')  # Reset pose
            rclpy.shutdown()
        elif key == 'robot':
            print('  current: %s' % robot_name)
            new_robot_name = input('  robot name? ')
            if new_robot_name != '':
                robot_name = new_robot_name
        elif key == '?' or key == 'help':
            self.print_help_messages()
            print('')

        # Arm stuffs
        elif key == 'X':
            axis = 'X'
        elif key == 'Y':
            axis = 'Y'
        elif key == 'Z':
            axis = 'Z'
        elif key == 'R':
            axis = 'Roll'
        elif key == 'P':
            axis = 'Pitch'
        elif key == 'W':
            axis = 'Yaw'
        elif key == '+':
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
        elif key == '-':
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
        elif _is_num(key):
            xyzrpy = self.xyzrpy_from_pose(self.get_current_pose(robot_name))
            print(xyzrpy)
            if axis == 'X':
                xyzrpy[0] = float(key)
            elif axis == 'Y':
                xyzrpy[1] = float(key)
            elif axis == 'Z':
                xyzrpy[2] = float(key)
            elif axis == 'Roll':
                xyzrpy[3] = float(key)
            elif axis == 'Pitch':
                xyzrpy[4] = float(key)
            else:
                xyzrpy[5] = float(key)
            self.go_to_pose_goal(robot_name,
                                 self.pose_from_xyzrpy(xyzrpy), speed=speed)
        elif key == 'home':
            self.go_to_named_pose(robot_name, 'home')
        elif key == 'back':
            self.go_to_named_pose(robot_name, 'back')
        elif key == 'named':
            pose_name = input('  pose name? ')
            try:
                self.go_to_named_pose(robot_name, pose_name, speed=speed)
            except rclpy.ROSException as e:
                self.get_logger().error('Unknown pose: %s' % e)
        elif key == 'frame':
            frame    = input('  frame? ')
            offset   = _get_offset()
            eef_link = input('  eef_link? ')
            try:
                self.go_to_frame(robot_name, frame, offset, speed=speed,
                                 end_effector_link=eef_link)
            except Exception as e:
                self.get_logger().error('Unknown frame: %s' % frame)
        elif key == 'clip':
            self.clip_wrist_joint_value(robot_name)
        elif key == 'speed':
           speed = float(input('  speed value? '))
        elif key == 'stop':
            self.stop(robot_name)
        elif key == 'jvalues':
            print(self.get_current_joint_values(robot_name))
        elif key == 'sm':
            self.spiral_motion(robot_name)
        elif key == 'SM':
            self.cancel_spiral_motion()

        # Gripper stuffs
        elif key == 'gripper':
            print('  current: %s' % self.gripper(robot_name).name)
            gripper_name = input('  gripper name? ')
            if gripper_name != '':
                try:
                    self.set_gripper(robot_name, gripper_name)
                except KeyError as e:
                    self.get_logger().error('Unknown gripper: %s' % e)
        elif key == 'pregrasp':
            self.pregrasp(robot_name)
        elif key == 'grasp':
            self.grasp(robot_name)
        elif key == 'postgrasp':
            self.postgrasp(robot_name)
        elif key == 'release':
            self.release(robot_name)
        elif key == 'gpos':
            position = float(input('  position? '))
            self.set_gripper_position(robot_name, position)
        elif key == 'gvel':
            position = float(input('  velocity? '))
            self.set_gripper_velocity(robot_name, position)
        elif key == 'tighten':
            self.tighten(robot_name, Duration(seconds=-1))
        elif key == 'loosen':
            self.loosen(robot_name, Duration(seconds=-1))
        elif key == 'gcancel':
            self.gripper_cancel(robot_name)

        # Gripper stuffs
        elif key == 'r':
            object_id   = input('  object_id? ')
            attach_link = input('  attach_link? ') if object_id == '' else ''
            self.com.remove_object(object_id, attach_link)

        else:
            print('  unknown command! [%s]' % key)
        return robot_name, axis, speed

    # Joint motion stuffs
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
            = JointTrajectory(joint_names=self.joint_names,
                              points=[
                                  JointTrajectoryPoint(
                                      positions=self.get_current_joint_values(robot_name),
                                      time_from_start=Duration(seconds=0)),
                                  JointTrajectoryPoint(
                                      positions=joint_values,
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

    # Cartesian motion stuffs
    def get_current_pose(self, robot_name):
        return self._cmd.get_group(robot_name).get_current_pose()

    def move_relative(self, robot_name, offset,
                      speed=1.0, accel=1.0, end_effector_link=''):
        return self.go_to_pose_goal(robot_name,
                                    self.get_current_pose(robot_name),
                                    offset, speed, accel, end_effector_link)

    def go_to_frame(self, robot_name, target_frame, offset=(),
                    speed=1.0, accel=1.0, end_effector_link=''):
        return self.go_to_pose_goal(robot_name,
                                    PoseStamped(
                                        header=Header(frame_id=target_frame),
                                        pose=Pose(
                                            position=Point(x=0, y=0, z=0),
                                            orientation=Quaternion(
                                                x=0, y=0, z=0, w=1))),
                                    offset, speed, accel, end_effector_link)

    def go_to_pose_goal(self, robot_name, target_pose, offset=(),
                        speed=1.0, accel=1.0, end_effector_link=''):
        return self.go_along_poses(robot_name,
                                   PoseArray(header=target_pose.header,
                                             poses=[target_pose.pose]),
                                   offset, speed, accel, end_effector_link)

    def go_along_poses(self, robot_name, poses, offset=(),
                       speed=1.0, accel=1.0, end_effector_link=''):
        group = self._cmd.get_group(robot_name)
        path  = self.create_path(robot_name, poses, offset,
                                 speed, accel, end_effector_link)
        if path is None:
            return False

        return self.execute_path(robot_name,
                                 group.retime_trajectory(
                                     self._cmd.get_current_state(), path,
                                     velocity_scaling_factor=speed,
                                     acceleration_scaling_factor=accel))

    def execute_path(self, robot_name, path):
        success = self._cmd.get_group(robot_name).execute(path, wait=True)
        if not success:
            self.get_logger().error('Failed to execute path.')
        self.stop(robot_name)
        return success

    def create_path(self, robot_name, poses, offset=(),
                    speed=1.0, accel=1.0, end_effector_link=''):
        group = self._cmd.get_group(robot_name)

        if end_effector_link == '':
            end_effector_link = self.gripper(robot_name).tip_link
        group.set_end_effector_link(end_effector_link)

        group.set_max_velocity_scaling_factor(np.clip(speed, 0.0, 1.0))
        group.set_max_acceleration_scaling_factor(np.clip(accel, 0.0, 1.0))
        transformed_poses = self.transform_poses_to_target_frame(poses, offset)

        try:
            path, fraction = group.compute_cartesian_path(
                                 transformed_poses.poses, self._eef_step, 0.0)
        except Exception as e:
            fraction = 0.0
            self.get_logger().error(e)

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

    # Gripper stuffs
    def default_gripper_name(self, robot_name):
        return self._default_gripper_names[robot_name]

    def set_gripper(self, robot_name, gripper_name):
        self._active_grippers[robot_name] = self._grippers[gripper_name]

    def gripper(self, robot_name):
        return self._active_grippers[robot_name]

    def set_gripper_parameters(self, robot_name, parameters):
        self.gripper(robot_name).parameters = parameters

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

    def tighten(self, robot_name, timeout=Duration()):
        self.gripper(robot_name).tighten(timeout)

    def loosen(self, robot_name, timeout=Duration()):
        self.gripper(robot_name).loosen(timeout)

    def gripper_cancel(self, robot_name):
        self.gripper(robot_name).cancel()

    # Camera stuffs
    def camera(self, camera_name):
        return self._cameras[camera_name]

    def continuous_shot(self, camera_name, enable):
        return self.camera(camera_name).continuous_shot(enable)

    def trigger_frame(self, camera_name):
        return self.camera(camera_name).trigger_frame()

    # Marker stuffs
    def delete_all_markers(self):
        self._markerPublisher.delete_all()

    def add_marker(self, marker_type, pose, endpoint=None,
                   text='', lifetime=15):
        self._markerPublisher.add(marker_type, pose, endpoint, text, lifetime)

    def publish_marker(self):
        self._markerPublisher.publish()

    # Graspability stuffs
    def create_mask_image(self, camera_name, nmasks):
        self.camera(camera_name).trigger_frame()
        return self._graspabilityClient.create_mask_image(nmasks)

    def graspability_send_goal(self, robot_name, part_id, mask_id,
                               one_shot=True):
        params = self._graspability_params[part_id]
        self._graspabilityClient.set_parameters(params)

        # Send goal first to be ready for subscribing image,
        self._graspabilityClient.send_goal(mask_id,
                                           self.gripper(robot_name).type,
                                           None if one_shot else
                                           self._graspability_feedback_cb)

    def graspability_cancel_goal(self):
        self._graspabilityClient.cancel_goal()

    def graspability_wait_for_result(self, target_frame='', pose_filter=None,
                                     marker_lifetime=0):
        graspabilities = self._graspabilityClient.wait_for_result()

        #  We have to transform the poses to reference frame before moving
        #  because graspability poses are represented w.r.t. camera frame
        #  which will change while moving in the case of "eye on hand".
        graspabilities.contact_points = self._transform_points_to_target_frame(
                                            graspabilities.poses.header,
                                            graspabilities.contact_points,
                                            target_frame)
        graspabilities.poses          = self.transform_poses_to_target_frame(
                                            graspabilities.poses, (),
                                            target_frame)
        if pose_filter is not None:
            poses          = []
            gscores        = []
            contact_points = []
            for pose, gscore, contact_point \
                in zip(graspabilities.poses.poses, graspabilities.gscores,
                       graspabilities.contact_points):
                filtered_pose = pose_filter(pose)
                if filtered_pose is not None:
                    poses.append(filtered_pose)
                    gscores.append(gscore)
                    contact_points.append(contact_point)
            graspabilities.poses.poses    = poses
            graspabilities.gscores        = gscores
            graspabilities.contact_points = contact_points
        self._graspability_publish_marker(graspabilities, marker_lifetime)
        self.get_logger().info('{} graspabilities found with stamp: [{:0>10}.{:0>9}]'
                      .format(len(graspabilities.poses.poses),
                              graspabilities.poses.header.stamp.secs,
                              graspabilities.poses.header.stamp.nsecs))
        return graspabilities

    def _graspability_publish_marker(self, graspabilities, marker_lifetime=0):
        self.delete_all_markers()
        for i, pose in enumerate(graspabilities.poses.poses):
            self.add_marker('graspability',
                            PoseStamped(graspabilities.poses.header, pose),
                            graspabilities.contact_points[i],
                            '{}[{:.3f}]'.format(i, graspabilities.gscores[i]),
                            lifetime=marker_lifetime)
        self.publish_marker()

    def _graspability_feedback_cb(self, feedback):
        self._graspability_publish_marker(feedback.graspabilities)

    # Pick and place action stuffs
    def pick(self, robot_name, target_pose, part_id,
             wait=True, done_cb=None, active_cb=None):
        params = self._picking_params.get(part_id)
        if params is None:
            params = self._picking_params[
                         self.com.get_object_info(part_id).object_type]
        if 'gripper_name' in params:
            self.set_gripper(robot_name, params['gripper_name'])
        if 'gripper_parameters' in params:
            self.set_gripper_parameters(robot_name,
                                        params['gripper_parameters'])
        return self._pick_or_place.send_goal(robot_name, target_pose, True,
                                             params['pick_offset'],
                                             params['approach_offset'],
                                             params['departure_offset'],
                                             params['speed_fast'],
                                             params['speed_slow'],
                                             '', wait, done_cb, active_cb)

    def place(self, robot_name, target_pose, part_id,
              subframe_link='', wait=True, done_cb=None, active_cb=None):
        params = self._picking_params.get(part_id)
        if params is None:
            params = self._picking_params[
                         self.com.get_object_info(part_id).object_type]
        placing_params = self._placing_params.get(target_pose.header.frame_id,
                                                  params)
        if not placing_params.get('place_offset'):
            placing_params = self._placing_params['default']
        if 'gripper_name' in params:
            self.set_gripper(robot_name, params['gripper_name'])
        if 'gripper_parameters' in params:
            self.set_gripper_parameters(robot_name,
                                        params['gripper_parameters'])
        return self._pick_or_place.send_goal(robot_name, target_pose, False,
                                             placing_params['place_offset'],
                                             placing_params['approach_offset'],
                                             placing_params['departure_offset'],
                                             params['speed_fast'],
                                             params['speed_slow'],
                                             subframe_link,
                                             wait, done_cb, active_cb)

    def pick_at_frame(self, robot_name, target_frame, part_id,
                      offset=(), wait=True, done_cb=None, active_cb=None):
        return self.pick(robot_name,
                         self.pose_from_xyzrpy(offset, target_frame),
                         part_id, wait, done_cb, active_cb)

    def place_at_frame(self, robot_name, target_frame, part_id,
                       offset=(), subframe_link='',
                       wait=True, done_cb=None, active_cb=None):
        return self.place(robot_name,
                          self.pose_from_xyzrpy(offset, target_frame),
                          part_id, subframe_link, wait, done_cb, active_cb)

    def pick_or_place_wait_for_stage(self, stage, timeout=Duration()):
        return self._pick_or_place.wait_for_stage(stage, timeout)

    def pick_or_place_wait_for_result(self, timeout=Duration()):
        if self._pick_or_place.wait_for_result(timeout):
            return self._pick_or_place.get_result().result

    def pick_or_place_cancel_goal(self):
        self._pick_or_place.cancel_goal()

    # Spiral motion action stuffs
    def spiral_motion(self, robot_name, end_effector_link='',
                      npoints=36, angle_increment=30.0,
                      radius_x_max=0.005, radius_y_max=0.0005,
                      speed=0.005, accel=1.0, timeout=Duration(seconds=30.0)):
        if end_effector_link == '':
            end_effector_link = self.gripper(robot_name).tip_link
        self._spiral_motion.send_goal(robot_name, end_effector_link,
                                      npoints, angle_increment,
                                      radius_x_max, radius_y_max,
                                      speed, accel, timeout)

    def cancel_spiral_motion(self):
        self._spiral_motion.cancel_goal()

    # Utility functions
    def transform_pose_to_target_frame(self, pose, offset=(), target_frame=''):
        poses = self.transform_poses_to_target_frame(
                    PoseArray(header=pose.header, poses=[pose.pose]),
                    offset, target_frame)
        return PoseStamped(header=poses.header, pose=poses.poses[0])

    def transform_poses_to_target_frame(self, poses,
                                        offset=(), target_frame=''):
        if target_frame == '':
            target_frame = self.reference_frame

        try:
            tfm = self._tf2_buffer.lookup_transform(target_frame,
                                                    poses.header.frame_id,
                                                    poses.header.stamp,
                                                    Duration(seconds=10)) \
                                  .transform
        except Exception as e:
            self.get_logger().error('AISTBaseRoutines.transform_poses_to_target_frame(): %s' % e)
            raise e

        transformed_poses = PoseArray(header=Header(frame_id=target_frame,
                                                    stamp=poses.header.stamp),
                                      poses=[])
        for pose in poses.poses:
            T = tfs.concatenate_matrices(
                    tfs.translation_matrix((tfm.translation.x,
                                            tfm.translation.y,
                                            tfm.translation.z)),
                    tfs.quaternion_matrix((tfm.rotation.x, tfm.rotation.y,
                                           tfm.rotation.z, tfm.rotation.w)),
                    tfs.translation_matrix((pose.position.x,
                                            pose.position.y,
                                            pose.position.z)),
                    tfs.quaternion_matrix((pose.orientation.x,
                                           pose.orientation.y,
                                           pose.orientation.z,
                                           pose.orientation.w)),
                    tfs.translation_matrix(self._position_from_offset(
                                                offset[0:3])),
                    tfs.quaternion_matrix(self._orientation_from_offset(
                                               offset[3:])))
            t = tfs.translation_from_matrix(T)
            q = tfs.quaternion_from_matrix(T)
            transformed_poses.poses.append(
                Pose(position=Point(x=t[0], y=t[1], z=t[2]),
                     orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])))
        return transformed_poses

    def lookup_pose(self, target_frame, source_frame):
        try:
            tfm = self._tf2_buffer.lookup_transform(target_frame,
                                                    source_frame, Time()) \
                                  .transform
        except Exception as e:
            self.get_logger().error('AISTBaseRoutines.lookup_pose(): %s' % e)
            return None
        return PoseStamped(header=Header(frame_id=target_frame),
                           pose=Pose(position=Point(x=tfm.translation.x,
                                                    y=tfm.translation.y,
                                                    z=tfm.translation.z),
                                     orientation=Quaternion(x=tfm.rotation.x,
                                                            y=tfm.rotation.y,
                                                            z=tfm.rotation.z,
                                                            w=tfm.rotation.w)))

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

    def pose_from_xyzrpy(self, xyzrpy=(), frame_id=''):
        if frame_id == '':
            frame_id = self.reference_frame
        t = self._position_from_offset(xyzrpy[0:3])
        q = self._orientation_from_offset(xyzrpy[3:])
        return PoseStamped(header=Header(frame_id=frame_id),
                           pose=Pose(position=Point(x=t[0], y=t[1], z=t[2]),
                                     orientation=Quaternion(x=q[0], y=q[1],
                                                            z=q[2], w=q[3])))

    def xyzrpy_from_pose(self, pose):
        transformed_pose = self.transform_pose_to_target_frame(pose).pose
        rpy = tfs.euler_from_quaternion((transformed_pose.orientation.x,
                                         transformed_pose.orientation.y,
                                         transformed_pose.orientation.z,
                                         transformed_pose.orientation.w))
        return [transformed_pose.position.x,
                transformed_pose.position.y,
                transformed_pose.position.z,
                degrees(rpy[0]), degrees(rpy[1]), degrees(rpy[2])]

    def format_pose(self, target_pose):
        return '[{:.4f}, {:.4f}, {:.4f}; {:.2f}, {:.2f}. {:.2f}]'.format(
            *self.xyzrpy_from_pose(target_pose))

    # Private functions
    def _initialize_collision_objects(self, initial_object_config):
        self.com.remove_object()
        # self.get_logger().info(initial_object_config)
        for object_type, config in initial_object_config.items():
            self.com.create_object(object_type,
                                   self.pose_from_xyzrpy(
                                       config.get('offset', ()),
                                       config['parent_link']),
                                   config.get('subframe', 'base_link'))
            time.sleep(0.5)
            # if object_type == 'panel_bearing' or object_type == 'panel_motor':
            #     self.com.attach_object(object_type, config['parent_link'])
            # if object_type == 'base':
            #     self.com.attach_object(object_type, config['parent_link'])

    def _position_from_offset(self, offset):
        return np.array((0.0, 0.0, 0.0) if len(offset) < 3 else offset[0:3])

    def _orientation_from_offset(self, offset):
        return np.array((0.0, 0.0, 0.0, 1.0)) if len(offset) < 3 else \
               tfs.quaternion_from_euler(*np.radians(offset[0:3])) if len(offset) == 3 else \
               np.array(offset[0:4])

    def _transform_points_to_target_frame(self, header, points,
                                          target_frame=''):
        if target_frame == '':
            target_frame = self._reference_frame

        try:
            self._tf2_buffer.lookup_transform(target_frame, header.frame_id,
                                              header.stamp,
                                              Duration(seconds=10))
            mat44 = self._tf2_buffer.asMatrix(target_frame, header)
        except Exception as e:
            self.get_logger().error('AISTBaseRoutines._transform_points_to_target_frame(): %s' % e)
            raise e

        return [ Point(*tuple(np.dot(mat44,
                                     np.array((p.x, p.y, p.z, 1.0)))[:3]))
                 for p in points ]

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
        return Quaternion(*tfs.quaternion_multiply(q, dq))
