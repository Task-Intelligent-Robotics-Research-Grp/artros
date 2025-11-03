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
import rclpy, time, copy, yaml, threading
import numpy as np
import tf_transformations as tfs

from rclpy.action                    import (ActionServer, ActionClient,
                                             GoalResponse, CancelResponse)
from rclpy.callback_groups           import MutuallyExclusiveCallbackGroup
from rclpy.wait_for_message          import wait_for_message
from geometry_msgs.msg               import PoseStamped
from aist_routines.base              import AISTBaseRoutines
from aist_msgs.action                import HandEyeCalibration
from aist_handeye_calibration.client import HandEyeCalibratorClient
from aist_utility.fileio             import filepath_from_url

######################################################################
#  class HandEyeCalibrationAction                                    #
######################################################################
class HandEyeCalibrationAction(object):
    class CancelRequestedException(Exception):
        def __init__(self):
            super().__init__()

    class AbortedException(Exception):
        def __init__(self):
            super().__init__()

    def __init__(self, node, calibrator_ns, server_ns):
        super().__init__()

        self._node        = node
        self._camera_name = node.declare_parameter('camera_name',
                                                   'a_motioncam').value
        self._robot_name  = node.declare_parameter('robot_name', 'b_bot').value
        self._eye_on_hand = node.declare_parameter('eye_on_hand', False).value
        self._end_effector_link = node.declare_parameter('end_effector_link',
                                                         'b_bot_flange').value
        self._calib_file  = node.declare_parameter('calib_file', '').value
        self._initpose    = node.declare_parameter('initpose', [0.0]).value
        self._keyposes    = node.declare_parameter('keyposes', [0.0]).value
        self._speed       = node.declare_parameter('speed', 1.0).value
        self._sleep_time  = node.declare_parameter('sleep_time', 2.0).value
        self._calibrator  = HandEyeCalibratorClient(node, calibrator_ns)

        # Action server
        self._server_cbg = MutuallyExclusiveCallbackGroup()
        self._server     = ActionServer(node, HandEyeCalibration, server_ns,
                                        execute_callback=self._execute_cb,
                                        callback_group=self._server_cbg,
                                        goal_callback=self._goal_cb,
                                        handle_accepted_callback=self._handle_accepted_cb,
                                        cancel_callback=self._cancel_cb)
        self._server_goal_handle = None
        self._goal_lock          = threading.Lock()

        # Action client
        self._client_goal_handle = None
        self._get_result_furue   = None
        self._client_cbg         = MutuallyExclusiveCallbackGroup()
        self._client             = ActionClient(node, HandEyeCalibration,
                                                server_ns,
                                                callback_group=self._client_cbg)
        self._client.wait_for_server()

    # Client stuffs
    @property
    def robot_name(self):
        return self._robot_name

    @property
    def speed(self):
        return self._speed

    def go_to_initpose(self):
        self._move(self._robot_name, self._initpose, self._end_effector_link)

    def go_to_marker(self):
        self._node.trigger_frame(self._camera_name)
        _, marker_pose = wait_for_message(PoseStamped, self._node, 'pose',
                                          time_to_wait=2.0)
        if marker_pose is None:
            self._node.get_logger().error('failed to detect marker')
            return False
        marker_pose = self._node.transform_pose_to_target_frame(marker_pose)
        success = self.go_to_pose_goal(self._robot_name,
                                       marker_pose, (0.0, 0.0, 0.05),
                                       speed=self._speed)
        print('  reached %s' %
              self.format_pose(self._node.get_current_pose(self._robot_name)))
        time.sleep(1.0)
        print('  move to %s' % self._node.format_pose(marker_pose))
        success = self.go_to_pose_goal(self._robot_name,
                                       marker_pose, speed=0.05)
        print('  reached %s' %
              self.format_pose(self._node.get_current_pose(self._robot_name)))

    def calibrate(self):
        self._get_result_future = None

        goal = HandEyeCalibration.Goal()
        goal.camera_name       = self._camera_name
        goal.robot_name        = self._robot_name
        goal.eye_on_hand       = self._eye_on_hand
        goal.end_effector_link = self._end_effector_link
        goal.initpose          = self._initpose
        goal.keyposes          = self._keyposes
        self._client.send_goal_async(goal) \
                    .add_done_callback(self._goal_response_cb)

    def wait(self):
        while self._get_result_future is None or \
              not self._get_result_future.done():
            time.sleep(0.1)
        return self._get_result_future.result().result.success

    def cancel(self):
        if not self._client_goal_handle:
            self._node.get_logger().warn('no active goals')
            return
        self._client_goal_handle.cancel_goal_async().add_done_callback(
            self._cancel_response_cb)

    def _goal_response_cb(self, future):
        self._client_goal_handle = future.result()
        if not self._client_goal_handle.accepted:
            self._node.get_logger().error('goal rejected')
            return
        self._node.get_logger().info('goal accepted')
        self._get_result_future = self._client_goal_handle.get_result_async()

    def _cancel_response_cb(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) == 0:
            self._node.get_logger().warn('no active goals')
        else:
            self._node.get_logger().info('goal canceled')

    # Server stuffs
    @property
    def current_goal(self):
        return self._server_goal_handle.request

    def _goal_cb(self, goal):
        self._node.get_logger().info('goal accepted')
        return GoalResponse.ACCEPT

    def _handle_accepted_cb(self, goal_handle):
        with self._goal_lock:
            if self._server_goal_handle is not None and \
               self._server_goal_handle.is_active:
                self._server_goal_handle.abort()
                self._node.get_logger().warn('previous goal aborted')
            self._server_goal_handle = goal_handle
        self._server_goal_handle.execute()

    def _cancel_cb(self, goal):
        self._node.get_logger().warn('goal requested to cancel')
        return CancelResponse.ACCEPT

    def _execute_cb(self, goal_handle):
        self._node.get_logger().info('executing goal...')

        try:
            result = HandEyeCalibration.Result()

            self._node.go_to_named_pose(self.current_goal.robot_name, 'home')
            self._move(self.current_goal.robot_name,
                       self.current_goal.initpose,
                       self.current_goal.end_effector_link)

            # Collect samples over pre-defined poses
            keyposes = np.array(self.current_goal.keyposes).reshape(-1, 6)\
                                                           .tolist()
            for i, keypose in enumerate(keyposes, 1):
                print('\n*** Keypose [%d/%d]: Try! ***' % (i, len(keyposes)))
                if self.current_goal.eye_on_hand:
                    self._move_to(keypose)
                else:
                    self._move_to_subposes(keypose, i)
                    print('*** Keypose [%d/%d]: Completed. ***'
                          % (i, len(keyposes)))

            res = self._calibrator.compute_calibration()

            self._save_calibration(res)
            self._node.go_to_named_pose(self.current_goal.robot_name, 'home')

            result.success = res.success
            with self._goal_lock:
                goal_handle.succeed()
                self._node.get_logger().info('goal succeeded')
        except HandEyeCalibrationAction.CancelRequestedException:
            result.success = False
            with self._goal_lock:
                goal_handle.canceled()
                self._node.get_logger().warn('goal canceled')
        except HandEyeCalibrationAction.AbortedException:
            result.success = False
            self._node.get_logger().warn('goal already aborted')
        except Exception as e:
            result.success = False
            with self._goal_lock:
                goal_handle.abort()
                self._node.get_logger()\
                          .error('goal aborted due to unexpected error: %s'
                                 % e)
        return result

    def _move_to_subposes(self, keypose, keypose_num):
        subpose = copy.copy(keypose)
        roll = subpose[3]
        for i in range(3):
            print('\n--- Subpose [%d/5]: Try! ---' % (i + 1))
            if self._move_to(subpose):
                self._node.get_logger().info('Subpose [%d/5]: Succeeded.'
                                             % (i + 1))
            else:
                self._node.get_logger().error('Subpose [%d/5]: Failed.'
                                              % (i + 1))
                subpose[3] -= 30.0

        subpose[3]  = roll - 30.0
        subpose[4] += 15.0

        for i in range(2):
            print('\n--- Subpose [%d/5]: Try! ---' % (i + 4))
            if self._move_to(subpose):
                self._node.get_logger().info('Subpose [%d/5]: Succeeded.'
                                             % (i + 4))
            else:
                self._node.get_logger().error('Subpose [%d/5]: Failed.'
                                              % (i + 4))
                subpose[4] -= 30.0

    def _move_to(self, xyzrpy):
        with self._goal_lock:
            if self._server_goal_handle.is_cancel_requested:
                raise HandEyeCalibrationAction.CancelRequestedException()
            if not self._server_goal_handle.is_active:
                raise HandEyeCalibrationAction.AbortedException()

        if not self._move(self.current_goal.robot_name, xyzrpy,
                          self.current_goal.end_effector_link):
            return False

        with self._goal_lock:
            if self._server_goal_handle.is_cancel_requested:
                raise HandEyeCalibrationAction.CancelRequestedException()
            if not self._server_goal_handle.is_active:
                raise HandEyeCalibrationAction.AbortedException()

        time.sleep(self._sleep_time)  # Wait for the robot to settle.
        future = self._calibrator.take_sample_async()
        self._node.trigger_frame(self.current_goal.camera_name)
        res = self._calibrator.wait_for_sample(future)
        if not res.success:
            self._node.get_logger().error('failed to take sample: %s'
                                          % res.message)
            return False

        self._node.get_logger().info('  %d-th sample taken'
                                     % len(self._calibrator.get_sample_list()\
                                           .transform_cm))
        return True

    def _move(self, robot_name, xyzrpy, end_effector_link):
        return self._node.go_to_pose_goal(robot_name,
                                          self._node.pose_from_xyzrpy(xyzrpy),
                                          end_effector_link=end_effector_link)

    def _save_calibration(self, res):
        def xyzrpy_from_transform(transform):
            rpy = tfs.euler_from_quaternion((transform.rotation.x,
                                             transform.rotation.y,
                                             transform.rotation.z,
                                             transform.rotation.w))
            return [transform.translation.x,
                    transform.translation.y,
                    transform.translation.z,
                    degrees(rpy[0]), degrees(rpy[1]), degrees(rpy[2])]

        if not res.success:
            self._node.get_logger().error('calibration failed: %s'
                                          % res.message)
            return

        print('=== estimated camera pose ===')
        print('[{:.4f}, {:.4f}, {:.4f}; {:.2f}, {:.2f}. {:.2f}]'\
              .format(*xyzrpy_from_transform(res.transform_ec.transform)))
        print('=== estimated marker pose ===')
        print('[{:.4f}, {:.4f}, {:.4f}; {:.2f}, {:.2f}. {:.2f}]'\
              .format(*xyzrpy_from_transform(res.transform_wm.transform)))
        print('trans. err(m): (mean, max) = (%f, %f)'
              % (res.mean_translation_error, res.max_translation_error))
        print('rot. err(deg): (mean, max) = (%f, %f)'
              % (res.mean_rotation_error, res.max_rotation_error))

        # Convert the transform to xyz-rpy representation.
        data = {'parent': res.transform_ec.header.frame_id,
                'child' : res.transform_ec.child_frame_id,
                'origin': xyzrpy_from_transform(res.transform_ec.transform)}

        # Save the transform.
        filename = filepath_from_url(self._calib_file)
        with open(filename, mode='w') as file:
            yaml.dump(data, file, default_flow_style=False)
        self.get_logger().info('saved calibration result in [%s]' % filename)
