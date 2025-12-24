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
import rclpy, threading
import numpy as np
import tf_transformations as tfs

from rclpy.action          import (ActionServer, ActionClient,
                                   GoalResponse, CancelResponse)
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from aist_msgs.action      import PickOrPlace
from geometry_msgs.msg     import Point, Quaternion, Pose, PoseStamped

######################################################################
#  class PickOrPlace                                                 #
######################################################################
class PickOrPlace(object):
    class Error(Exception):
        def __init__(self, result, text):
            super().__init__(text)
            self.result = result

    def __init__(self, node, server_ns='pick_or_place'):
        super().__init__()

        self._node      = node
        self._current_stage = PickOrPlaceFeedback.IDLING
        self._target_stage  = None
        self._condition     = threading.Condition()

        # Action server
        self._server_cbg = MutuallyExclusiveCallbackGroup()
        self._server     = ActionServer(
                               node, PickOrPlace, server_ns,
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
        self._client     = ActionClient(node, PickOrPlace, server_ns,
                                        callback_group=self._client_cbg)
        self._client.wait_for_server()

    @property
    def current_stage(self):
        return self._current_stage

    @property
    def _logger(self):
        return self._node.get_logger()

    # Client stuffs
    def send_goal(self, robot_name, pose, pick, offset,
                  approach_offset, departure_offset, speed_fast, speed_slow,
                  subframe_link='', wait=True, done_cb=None, active_cb=None):
        self._get_result_future = None
        self._current_stage     = PickOrPlace.Feedback.IDLING
        self._target_stage      = PickOrPlace.Feedback.IDLING

        goal = PickOrPlace.Goal()
        goal.robot_name       = robot_name,
        goal.subframe_link    = subframe_link
        goal.pose             = pose
        goal.pick             = pick
        goal.offset           = offset
        goal.approach_offset  = approach_offset
        goal.departure_offset = departure_offset
        goal.speed_fast       = speed_fast
        goal.speed_slow       = speed_slow
        self._client.send_goal_async(goal) \
                    .add_done_callback(self._goal_response_cb)
        if wait:
            self.wait()
            return self.get_result().result

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

    def wait_for_stage(self, stage, timeout=rospy.Duration()):
        self._target_stage = stage          # Set stage to be waited for
        timeout_time = rospy.get_rostime() + timeout
        loop_period  = rospy.Duration(0.1)
        with self._condition:
            # Loop to avoid spurious wakeup
            while self._current_stage != self._target_stage:
                time_left = timeout_time - rospy.get_rostime()
                if timeout   >  rospy.Duration(0.0) and \
                   time_left <= rospy.Duration(0.0):
                    return False            # Timeout has expired
                if time_left > loop_period or timeout == rospy.Duration():
                    time_left = loop_period
                self._condition.wait(time_left.to_sec())
        return True

    def _feedback_cb(self, feedback):
        self._current_stage = feedback.stage
        if self._current_stage == self._target_stage:
            with self._condition:
                self._condition.notifyAll()

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
        request = goal_handle.request
        self._logger.loginfo('### Do %s ###'
                             % 'picking' if request.pick else 'placing')
        node    = self._node
        com     = node.com
        gripper = node.gripper(request.robot_name)
        if request.pick:
            object_id = PickOrPlace._get_object_id(request.pose.header.frame_id)
            eef_link  = ''
        else:
            object_id = PickOrPlace._get_object_id(request.subframe_link)
            eef_link  = request.subframe_link

        try:
            # Go to approach pose.
            self._publish_feedback(PickOrPlaceFeedback.MOVING,
                                   'Go to approach pose')
            speed   = request.speed_fast if request.pick else request.speed_slow
            success = node.go_to_pose_goal(request.robot_name, request.pose,
                                               request.approach_offset, speed,
                                               end_effector_link=eef_link)

            # Check success of going to approach pose.
            if not self._server.is_active():
                return
            if not success:
                raise PickOrPlace.Error(PickOrPlaceResult.MOVE_FAILURE,
                                        'Failed to go to approach pose')

            # Go to pick/place pose.
            self._publish_feedback(PickOrPlaceFeedback.APPROACHING,
                                   'Go to %s pose' %
                                   ('pick' if request.pick else 'place'))
            if request.pick:
                gripper.pregrasp()  # Pregrasp (not wait)
                gripper.wait()      # Wait for pregrasp completed
                if object_id != '':
                    com.allow_collision(object_id, gripper.tip_link)
            elif object_id != '':
                com.append_touch_links(object_id, request.pose.header.frame_id)

            success = node.go_to_pose_goal(request.robot_name, request.pose,
                                               request.offset, request.speed_slow,
                                               end_effector_link=eef_link)

            # Check success of going to pick/place pose.
            if not self._server.is_active():
                return
            if not success:
                if request.pick:
                    gripper.release()
                raise PickOrPlace.Error(PickOrPlaceResult.APPROACH_FAILURE,
                                        'Failed to approach target')

            # Grasp/release at pick/place pose.
            self._publish_feedback(PickOrPlaceFeedback.GRASPING_OR_RELEASING,
                                   'Pick' if request.pick else 'Place')
            if request.pick:
                if 'spiral_npoints' in gripper.parameters:
                    timeout = rospy.Duration(
                                  gripper.parameters['spiral_timeout'])
                    node.spiral_motion(
                        request.robot_name, gripper.tip_link,
                        gripper.parameters['spiral_npoints'],
                        gripper.parameters['spiral_angle_increment'],
                        gripper.parameters['spiral_radius_x_max'],
                        gripper.parameters['spiral_radius_y_max'],
                        gripper.parameters['spiral_speed'],
                        gripper.parameters['spiral_accel'],
                        timeout)
                    if gripper.grasp(timeout):
                        node.cancel_spiral_motion()
                        rospy.sleep(rospy.Duration(0.5))
                else:
                    gripper.grasp()
                if object_id != '':
                    original_object_info = com.attach_object(object_id,
                                                             gripper.tip_link)
            else:
                gripper.release()
                if object_id != '':
                    inhand_pose = node.lookup_pose(request.subframe_link,
                                                       gripper.tip_link)
                    com.detach_object(object_id, request.pose.header.frame_id,
                                      PickOrPlace._get_object_id(
                                          gripper.tip_link))

            # Go back to departure(pick) or approach(place) pose.
            self._publish_feedback(PickOrPlaceFeedback.DEPARTING,
                                   'Go back to departure pose')
            if request.pick:
                gripper.postgrasp()                 # Postgrasp (not wait)
                speed  = request.speed_slow
                pose   = request.pose
                offset = request.departure_offset
            else:
                speed = request.speed_fast
                if object_id != '':
                    pose   = PoseStamped(request.pose.header,
                                         PickOrPlace._concatenate_poses(
                                             request.pose.pose,
                                             node.pose_from_xyzrpy(
                                                 request.departure_offset).pose,
                                             inhand_pose.pose))
                    offset = ()
                else:
                    pose   = request.pose
                    offset = request.approach_offset
            success = node.go_to_pose_goal(request.robot_name,
                                               pose, offset, speed)

            # Check success of going back to departure/approach pose.
            if not self._server.is_active():
                return
            if not success:
                if request.pick:
                    gripper.release()
                    if object_id != '':
                        com.detach_object(object_id,
                                          original_object_info.parent_link,
                                          PickOrPlace._get_object_id(
                                              gripper.tip_link))
                raise PickOrPlace.Error(PickOrPlaceResult.DEPARTURE_FAILURE,
                                        'Failed to depart from target')

            # Check success of postgrasp.
            if request.pick and \
               rospy.get_param('use_real_robot', False) and \
               not gripper.wait():  # Wait for postgrasp completed
                gripper.release()
                if object_id != '':
                    com.detach_object(object_id,
                                      original_object_info.parent_link,
                                      PickOrPlace._get_object_id(
                                          gripper.tip_link))
                    com.move_object(object_id, original_object_info.pose,
                                    PickOrPlace._get_subframe(
                                        request.pose.header.frame_id))
                raise PickOrPlace.Error(PickOrPlaceResult.GRASP_FAILURE,
                                        'Failed to grasp')

            self._server.set_succeeded(PickOrPlaceResult(
                                           PickOrPlaceResult.SUCCESS))
            rospy.loginfo('### %s succeeded. ###',
                          'Pick' if request.pick else 'Place')
        except PickOrPlace.Error as err:
            self._server.set_aborted(PickOrPlaceResult(err.result), str(err))
            self._logger.error('### %s aborted[%s]. ###'
                               % ('Pick' if request.pick else 'Place', err))
        finally:
            if object_id != '':
                #com.disallow_collision(object_id, gripper.tip_link)
                com.reset_touch_links()

    def _preempt_cb(self):
        goal = self._server.current_goal.get_goal()
        self._node.stop(goal.robot_name)
        self._node.gripper(goal.robot_name).release()
        self._server.set_preempted(PickOrPlaceResult(
                                       PickOrPlaceResult.PREEMPTED))
        self._logger.warn('### %s cancelled. ###'
                          % 'Pick' if goal.pick else 'Place')

    def _publish_feedback(self, stage, text):
        self._server.publish_feedback(PickOrPlace.Feedback(stage))
        self._logger.info('--- %s ---' % text)

    def _set_aborted(self, result, text):
        goal = self._server.current_goal.get_goal()
        self._server.set_aborted(PickOrPlaceResult(result))
        self._logger.error('### %s aborted: %s ###',
                           % ('Pick' if goal.pick else 'Place', text))

    @staticmethod
    def _get_object_id(link_name):
        tokens = link_name.rsplit('/', 1)
        return tokens[0] if len(tokens) == 2 else ''

    @staticmethod
    def _get_subframe(link_name):
        tokens = link_name.rsplit('/', 1)
        return tokens[1] if len(tokens) == 2 else link_name

    @staticmethod
    def _concatenate_poses(*poses):
        T = np.identity(4)
        for pose in poses:
            T = T @ tfs.translation_matrix((pose.position.x,
                                            pose.position.y,
                                            pose.position.z)) \
                  @ tfs.quaternion_matrix((pose.orientation.x,
                                           pose.orientation.y,
                                           pose.orientation.z,
                                           pose.orientation.w))
        return Pose(Point(*tfs.translation_from_matrix(T)),
                    Quaternion(*tfs.quaternion_from_matrix(T)))
