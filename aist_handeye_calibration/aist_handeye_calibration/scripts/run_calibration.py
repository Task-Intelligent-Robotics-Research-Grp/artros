#!/usr/bin/env python3
#
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
import rclpy, sys, time, copy, yaml, threading
import numpy as np
import tf_transformations as tfs

from rclpy.action                         import ActionClient
from rclpy.duration                       import Duration
from rclpy.executors                      import MultiThreadedExecutor
from rclpy.callback_groups                import MutuallyExclusiveCallbackGroup
from std_srvs.srv                         import Empty, Trigger
from geometry_msgs.msg                    import (PoseStamped, Pose, Point,
                                                  Quaternion)
from action_msgs.msg                      import GoalStatus
from aist_routines.base                   import AISTBaseRoutines
from aist_handeye_calibration_msgs.srv    import (GetSampleList,
                                                  ComputeCalibration)
from aist_handeye_calibration_msgs.action import TakeSample
from aist_utility.fileio                  import filepath_from_url

######################################################################
#  class HandEyeCalibrationRoutines                                  #
######################################################################
class HandEyeCalibrationRoutines(AISTBaseRoutines):
    def __init__(self, name):
        super().__init__(name)

        self._camera_name = self.declare_parameter('camera_name',
                                                   'a_motioncam').value
        self._robot_name  = self.declare_parameter('robot_name', 'b_bot').value
        self._eye_on_hand = self.declare_parameter('eye_on_hand', False).value
        self._robot_effector_frame \
            = self.declare_parameter('robot_effector_frame',
                                     'b_bot_flange').value
        self._robot_effector_tip_frame \
            = self.declare_parameter('robot_effector_tip_frame', '').value
        self._initpose   = self.declare_parameter('initpose', [0.0])\
                               .get_parameter_value().double_array_value
        self._keyposes   = np.array(self.declare_parameter('keyposes', [0.0])\
                                    .get_parameter_value().double_array_value)\
                             .reshape(-1, 6).tolist()
        self._speed      = self.declare_parameter('speed', 1.0).value
        self._sleep_time = self.declare_parameter('sleep_time', 2.0).value

        if self.declare_parameter('calibration', True).value:
            ns = 'handeye_calibrator'
            self._cbg                 = MutuallyExclusiveCallbackGroup()
            self._get_sample_list     = self.create_client(
                                            GetSampleList,
                                            ns + '/get_sample_list',
                                            callback_group=self._cbg)
            self._compute_calibration = self.create_client(
                                            ComputeCalibration,
                                            ns + '/compute_calibration',
                                            callback_group=self._cbg)
            self._save_calibration    = self.create_client(
                                            Trigger, ns + '/save_calibration',
                                            callback_group=self._cbg)
            self._reset               = self.create_client(
                                            Empty, ns + '/reset',
                                            callback_group=self._cbg)
            self._take_sample_cbg     = MutuallyExclusiveCallbackGroup()
            self._take_sample         = ActionClient(self, TakeSample,
                                                     ns + '/take_sample',
                                                     callback_group=self._take_sample_cbg)
        else:
            self._get_sample_list     = None
            self._compute_calibration = None
            # self._save_calibration    = None
            self._reset               = None
            self._take_sample         = None

        cli_thread = threading.Thread(target=self.run)
        cli_thread.daemon = True
        cli_thread.start()

    def run(self):
        # Reset pose
        self.go_to_named_pose(self._robot_name, "home")
        self.print_help_messages()
        print('')

        axis = 'Y'

        while rclpy.ok():
            prompt = '{:>5}:{}>> '.format(axis, self.format_pose(
                                                    self.get_current_pose(
                                                        self._robot_name)))
            key = input(prompt)
            _, axis, _ = self.interactive(key, self._robot_name, axis,
                                          self._speed)
        self.destroy_node()
        rclpy.shutdown()

    # interactive stuffs
    def print_help_messages(self):
        super(HandEyeCalibrationRoutines, self).print_help_messages()
        print('=== Calibration commands ===')
        print('  init:  go to initial pose')
        print('  calib: do calibration')
        print('  check: go to marker')

    def interactive(self, key, robot_name, axis, speed):
        if key == 'init':
            self.go_to_initpose()
        elif key == 'calib':
            self.calibrate()
        elif key == 'check':
            self.go_to_marker()
        else:
            return super().interactive(key, robot_name, axis, speed)
        return robot_name, axis, speed

    def go_to_initpose(self):
        print('initpose=%s' % self._initpose)
        self._move(self._initpose)

    def calibrate(self):
        # if self._reset:
        #     self._reset.call(Empty.Request())

        # Reset pose
        self.go_to_named_pose(self._robot_name, 'home')
        #self.go_to_initpose()

        # Collect samples over pre-defined poses
        keyposes = self._keyposes
        for i, keypose in enumerate(keyposes, 1):
            print('\n*** Keypose [%d/%d]: Try! ***' % (i, len(keyposes)))
            if self._eye_on_hand:
                self._move_to(keypose, i, 1)
            else:
                self._move_to_subposes(keypose, i)
            print('*** Keypose [%d/%d]: Completed. ***' % (i, len(keyposes)))

        if self._compute_calibration:
            try:
                res = self._compute_calibration.call(
                          ComputeCalibration.Request())
                print(res.message)
                if res.success:
                    self._save_camera_placement(res.Tec)
                    res = self._save_calibration.call(
                              SaveCalibration.Request())
                    print(res.message)
            except Exception as e:
                self.get_logger().error(e)
        self.go_to_named_pose(self._robot_name, 'home')

    def go_to_marker(self):
        self.trigger_frame(self._camera_name)
        try:
            marker_pose = rclpy.wait_for_message(PoseStamped, self,
                                                 '/aruco_detector_3d/pose')
        except Exception as e:
            self.get_logger().error(e)
            return

        #  We must transform the marker pose to reference frame before moving
        #  to the approach pose because the marker pose is given w.r.t. camera
        #  frame which will change while moving in the case of "eye on hand".
        marker_pose = self.transform_pose_to_target_frame(marker_pose)
        success = self.go_to_pose_goal(self._robot_name,
                                       marker_pose, (0, 0, 0.05),
                                       speed=self._speed,
                                       end_effector_link=self._robot_effector_tip_frame)
        print('  reached %s' %
              self.format_pose(self.get_current_pose(self._robot_name)))
        time.sleep(1.0)
        print('  move to %s' % self.format_pose(marker_pose))
        success = self.go_to_pose_goal(self._robot_name,
                                       marker_pose, speed=0.05,
                                       end_effector_link=self._robot_effector_tip_frame)
        print('  reached %s' %
              self.format_pose(self.get_current_pose(self._robot_name)))

    # Move stuffs
    def _move_to_subposes(self, keypose, keypose_num):
        subpose = copy.copy(keypose)
        roll = subpose[3]
        for i in range(3):
            print('\n--- Subpose [%d/5]: Try! ---' % (i + 1))
            if self._move_to(subpose, keypose_num, i + 1):
                self.get_logger().info('Subpose [%d/5]: Succeeded.' % (i + 1))
            else:
                self.get_logger().error('Subpose [%d/5]: Failed.' % (i + 1))
            subpose[3] -= 30

        subpose[3] = roll - 30
        subpose[4] += 15

        for i in range(2):
            print('\n--- Subpose [%d/5]: Try! ---' % (i + 4))
            if self._move_to(subpose, keypose_num, i + 4):
                self.get_logger().info('Subpose [%d/5]: Succeeded.' % (i + 4))
            else:
                self.get_logger().error('Subpose [%d/5]: Failed.' % (i + 4))
            subpose[4] -= 30

    def _move_to(self, subpose, keypose_num, subpose_num):
        if not self._move(subpose):
            return False

        if self._take_sample:
            time.sleep(self._sleep_time)  # Wait for the robot to settle.
            self._send_goal()
            self.trigger_frame(self._camera_name)
            result = self._wait_for_result(Duration(seconds=3))
            if  result is None:
#                self._goal_handle.cancel_goal_async()  # timeout expired
                self.get_logger().error('TakeSampleAction: timeout expired')
                return False
            if self._goal_handle.status != GoalStatus.SUCCEEDED:
                self.get_logger().error(
                    'TakeSampleAction: not in succeeded state')
                return False

            pose = PoseStamped()
            pose.header = result.Tcm.header
            pose.pose.position    = result.Tcm.transform.translation
            pose.pose.orientation = result.Tcm.transform.rotation
#            print('  camera <= marker   %s' % self.format_pose(pose))
            pose.header = result.Twe.header
            pose.pose.position    = result.Twe.transform.translation
            pose.pose.orientation = result.Twe.transform.rotation
            print('  world  <= effector %s' % self.format_pose(pose))

            n = len(self._get_sample_list.call(GetSampleList.Request()).Tcm)
            print('  %d samples taken' % n)

        return True

    def _move(self, xyzrpy):
        pose = self.pose_from_xyzrpy(xyzrpy)
        print('  move to %s' % self.format_pose(pose))
        success = self.go_to_pose_goal(self._robot_name, pose,
                                       speed=self._speed,
                                       end_effector_link=self._robot_effector_frame)
        print('  reached %s' %
              self.format_pose(self.get_current_pose(self._robot_name)))
        return success

    def _save_camera_placement(self, Tec):
        # Frame to which the camera attached
        camera_parent_frame = self.get_param('~camera_parent_frame')

        # Get camera base frame whose parent is camera_parent_frame.
        camera_frame      = Tec.child_frame_id
        stamp             = Tec.header.stamp
        chain             = self.listener.chain(camera_parent_frame, stamp,
                                                camera_frame, stamp,
                                                camera_parent_frame)
        camera_base_frame = chain[-2]

        # Compute transform from camera base frame to its parent.
        Mec = self.listener.fromTranslationRotation(
                                (Tec.transform.translation.x,
                                 Tec.transform.translation.y,
                                 Tec.transform.translation.z),
                                (Tec.transform.rotation.x,
                                 Tec.transform.rotation.y,
                                 Tec.transform.rotation.z,
                                 Tec.transform.rotation.w))
        Mpe = self.listener.fromTranslationRotation(
                                *self.listener.lookupTransform(
                                    camera_parent_frame,
                                    Tec.header.frame_id, stamp))
        Mcb = self.listener.fromTranslationRotation(
                                *self.listener.lookupTransform(
                                    camera_frame, camera_base_frame, stamp))
        Mpb = tfs.concatenate_matrices(Mpe, Mec, Mcb)

        # Convert the transform to xyz-rpy representation.
        xyz  = list(map(float, tfs.translation_from_matrix(Mpb)))
        rpy  = list(map(float, tfs.euler_from_matrix(Mpb)))
        data = {'parent': camera_parent_frame,
                'child' : camera_base_frame,
                'origin': xyz + rpy}

        # Save the transform.
        filename = filepath_from_url('package://aist_handeye_calibration/calib/' + self._camera_name + '.yaml')
        with open(filename, mode='w') as file:
            yaml.dump(data, file, default_flow_style=False)
            self.get_logger().info('Saved transform from camera base frame[%s] to camera parent frame[%s] into %s'
                                   % (camera_base_frame,
                                      camera_parent_frame, filename))

    def _send_goal(self):
        self._goal_handle = None
        self._get_result_future = None
        self._take_sample.send_goal_async(TakeSample.Goal()) \
            .add_done_callback(self._goal_response_cb)
        self.get_logger().info('### goal sent to the server')

    def _wait_for_result(self, timeout):
        timeout_time = self.get_clock().now() + timeout
        while self._get_result_future is None or \
              not self._get_result_future.done():
            if self.get_clock().now() > timeout_time:
                self.get_logger().error('timeout[%.1fs] has expired before goal finised' % (timeout.nanoseconds * 1.0e-9))
                return None
            time.sleep(0.1)
        return self._get_result_future.result()

    def _goal_response_cb(self, future):
        self.get_logger().info('### goal_response_cb()')
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().error('goal rejected')
            return
        self.get_logger().info('goal accepted')
        self._get_result_future = self._goal_handle.get_result_async()

######################################################################
#  global functions                                                  #
######################################################################
def main():
    rclpy.init(args=sys.argv)

    node = HandEyeCalibrationRoutines('run_calibration')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

if __name__ == '__main__':
    main()
