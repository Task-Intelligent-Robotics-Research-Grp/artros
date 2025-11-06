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

from math                           import degrees
from rclpy.action                   import (ActionServer, ActionClient,
                                            GoalResponse, CancelResponse)
from rclpy.callback_groups          import MutuallyExclusiveCallbackGroup
from rclpy.wait_for_message         import wait_for_message
from geometry_msgs.msg              import PoseStamped
from aist_routines.base             import AISTBaseRoutines
from aist_msgs.action               import CameraCalibration
from aist_camera_calibration.client import CameraCalibratorClient
from aist_utility.fileio            import filepath_from_url
from camera_info_manager\
    .camera_info_manager            import saveCalibration

######################################################################
#  class CameraCalibrationAction                                     #
######################################################################
class CameraCalibrationAction(object):
    class CancelRequestedException(Exception):
        def __init__(self):
            super().__init__()

    class AbortedException(Exception):
        def __init__(self):
            super().__init__()

    def __init__(self, node, calibrator_ns, server_ns):
        super().__init__()

        self._node              = node
        self._robot_name        = node.declare_parameter(
                                      'robot_name', 'b_bot').value
        self._end_effector_link = node.declare_parameter(
                                      'end_effector_link',
                                      'b_bot_flange').value
        self._calib_dir         = node.declare_parameter(
                                      'calibration_dir', '').value
        self._speed             = node.declare_parameter(
                                      'speed', 1.0).value
        self._settling_time     = node.declare_parameter(
                                      'settling_time', 2.0).value
        self._initpose          = node.declare_parameter(
                                      'initpose', [0.0]).value
        self._keyposes          = node.declare_parameter(
                                      'keyposes', [0.0]).value

        # Service clients
        self._calibrator = CameraCalibratorClient(node, calibrator_ns)

        # Action server
        self._server_cbg = MutuallyExclusiveCallbackGroup()
        self._server     = ActionServer(
                               node, CameraCalibration, server_ns,
                               execute_callback=self._execute_cb,
                               callback_group=self._server_cbg,
                               goal_callback=self._goal_cb,
                               handle_accepted_callback=self._handle_accepted_cb,
                               cancel_callback=self._cancel_cb)
        self._server_gh  = None
        self._goal_lock  = threading.Lock()

        # Action client
        self._client_gh  = None
        self._get_result_future = None
        self._client_cbg = MutuallyExclusiveCallbackGroup()
        self._client     = ActionClient(
                               node, CameraCalibration, server_ns,
                               callback_group=self._client_cbg)
        self._client.wait_for_server()

    @property
    def robot_name(self):
        return self._robot_name

    @property
    def speed(self):
        return self._speed

    @property
    def _logger(self):
        return self._node.get_logger()

    def go_to_initpose(self):
        self._move(self._robot_name, self._initpose, self._end_effector_link)

    def get_sample_list(self):
        return self._calibrator.get_sample_list()

    def reset(self):
        return self._calibrator.reset()

    # Client stuffs
    def calibrate(self):
        self._get_result_future = None

        goal = CameraCalibration.Goal()
        goal.robot_name        = self._robot_name
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
        if not self._client_gh:
            self._logger.warn('no active goals')
            return
        self._client_gh.cancel_goal_async().add_done_callback(
            self._cancel_response_cb)

    def _goal_response_cb(self, future):
        self._client_gh = future.result()
        if not self._client_gh.accepted:
            self._logger.error('goal rejected')
            return
        self._logger.info('goal accepted')
        self._get_result_future = self._client_gh.get_result_async()

    def _cancel_response_cb(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) == 0:
            self._logger.warn('no active goals')
        else:
            self._logger.info('goal canceled')

    # Server stuffs
    def _goal_cb(self, goal):
        self._logger.info('goal accepted')
        return GoalResponse.ACCEPT

    def _handle_accepted_cb(self, goal_handle):
        with self._goal_lock:
            if self._server_gh is not None and \
               self._server_gh.is_active:
                self._server_gh.abort()
                self._logger.warn('previous goal aborted')
            self._server_gh = goal_handle
        self._server_gh.execute()

    def _cancel_cb(self, goal):
        self._logger.warn('goal requested to cancel')
        return CancelResponse.ACCEPT

    def _execute_cb(self, goal_handle):
        self._logger.info('executing goal...')

        try:
            result = CameraCalibration.Result()

            self._calibrator.reset()
            self._node.go_to_named_pose(goal_handle.request.robot_name, 'home')
            self._move(goal_handle.request.robot_name,
                       goal_handle.request.initpose,
                       goal_handle.request.end_effector_link)

            # Collect samples over pre-defined poses
            keyposes = np.array(goal_handle.request.keyposes).reshape(-1, 6)\
                                                             .tolist()
            for i, keypose in enumerate(keyposes, 1):
                print('\n*** Keypose [%d/%d]: Try! ***' % (i, len(keyposes)))
                self._move_to(goal_handle, keypose)
                print('*** Keypose [%d/%d]: Completed. ***'
                      % (i, len(keyposes)))

            res = self._calibrator.compute_calibration()

            self._save_calibration(res)
            self._node.go_to_named_pose(goal_handle.request.robot_name, 'home')

            result.success = res.success
            with self._goal_lock:
                goal_handle.succeed()
            self._logger.info('goal succeeded')
        except CameraCalibrationAction.CancelRequestedException:
            result.success = False
            with self._goal_lock:
                goal_handle.canceled()
            self._logger.warn('goal canceled')
        except CameraCalibrationAction.AbortedException:
            result.success = False
            self._logger.warn('goal already aborted')
        # except Exception as e:
        #     result.success = False
        #     with self._goal_lock:
        #         goal_handle.abort()
        #     self._logger.error('goal aborted due to unexpected error: %s'
        #                            % e)
        return result

    def _move_to(self, goal_handle, xyzrpy):
        with self._goal_lock:
            if goal_handle.is_cancel_requested:
                raise CameraCalibrationAction.CancelRequestedException()
            if not goal_handle.is_active:
                raise CameraCalibrationAction.AbortedException()

        if not self._move(goal_handle.request.robot_name, xyzrpy,
                          goal_handle.request.end_effector_link):
            return False

        with self._goal_lock:
            if goal_handle.is_cancel_requested:
                raise CameraCalibrationAction.CancelRequestedException()
            if not goal_handle.is_active:
                raise CameraCalibrationAction.AbortedException()

        time.sleep(self._settling_time)  # Wait for the robot to settle.
        future = self._calibrator.take_sample_async()
        #self._node.trigger_frame(goal_handle.request.camera_name)
        res = self._calibrator.wait_for_sample(future)
        if not res.success:
            self._logger.error('failed to take sample: %s' % res.message)
            return False

        self._logger.info('  %d-th sample taken'
                          % len(self._calibrator.get_sample_list()\
                                .correspondences_sets))
        return True

    def _move(self, robot_name, xyzrpy, end_effector_link):
        return self._node.go_to_pose_goal(robot_name,
                                          self._node.pose_from_xyzrpy(xyzrpy),
                                          end_effector_link=end_effector_link)

    def _save_calibration(self, res):
        if not res.success:
            self._logger.error('calibration failed: %s' % res.message)
            return

        for camera_name, camera_info, camera_pose in zip(res.camera_names,
                                                         res.intrinsics,
                                                         res.camera_poses):
            print('=== estimated pose of %s ===' % camera_name)
            print('[{:.4f}, {:.4f}, {:.4f}; {:.2f}, {:.2f}. {:.2f}]'\
                  .format(*self._node.xyzrpy_from_pose(camera_pose)))

            # Convert camera pose to xyz-rpy representation.
            data = {'parent': camera_pose.header.frame_id,
                    'child' : camera_info.header.frame_id,
                    'origin': self._node.xyzrpy_from_pose(camera_pose)}

            # Save camera pose.
            dirname  = filepath_from_url(self._calib_dir)
            filename = dirname + '/' + camera_name + '.yaml'
            with open(filename, mode='w') as file:
                yaml.dump(data, file, default_flow_style=False)
            self._logger.info('saved camera extrinsiscs in [%s]' % filename)

            # Save camera_info.
            filename = dirname + '/' + camera_name + '-camera_info.yaml'
            print('*** %s' % camera_info)
            saveCalibration(camera_info, filename, camera_name)
            self._logger.info('saved camera intrinsiscs in [%s]' % filename)

        print('=== reprojection error: %f(pix) ===' % res.error)
