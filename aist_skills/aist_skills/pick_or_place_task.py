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

from rclpy.action                import GoalResponse, CancelResponse
from rclpy.callback_groups       import MutuallyExclusiveCallbackGroup
from aist_msgs.action            import PickOrPlace
from geometry_msgs.msg           import Point, Quaternion, Pose, PoseStamped
from task_wrappers.action_server import ActionServer
from task_wrappers.action_client import GroupedSimpleActionClient

#************************************************************************
#  class PickOrPlaceTaskClient                                          *
#************************************************************************
class PickOrPlaceTaskClient(GroupedSimpleActionClient):
    def __init__(self, node, server_ns='pick_or_place'):
        self._client_cbg = MutuallyExclusiveCallbackGroup()
        super().__init__(node, PickOrPlace, server_ns,
                         callback_group=self._client_cbg,
                         group_field='robot_name')
        self.wait_for_server()

    def send_goal(self, robot_name, pose, pick, offset,
                  approach_offset, departure_offset, speed_fast, speed_slow,
                  subframe_link='', timeout_sec=None):
        return super().send_goal(PickOrPlace.Goal(
                                     robot_name=robot_name,
                                     subframe_link=subframe_link,
                                     pose=pose,
                                     pick=pick,
                                     offset=offset,
                                     approach_offset=approach_offset,
                                     departure_offset=departure_offset,
                                     speed_fast=speed_fast,
                                     speed_slow=speed_slow),
                                 feedback_callback=self.stage_feedback_cb,
                                 timeout_sec=timeout_sec)

#************************************************************************
#  class PickOrPlaceTaskServer                                          *
#************************************************************************
class PickOrPlaceTaskServer(ActionServer):
    class _Error(Exception):
        def __init__(self, result, stage, text):
            super().__init__(text)
            self.result = result
            self.stage  = stage

    def __init__(self, node, server_ns='pick_or_place'):
        self._server_cbg = MutuallyExclusiveCallbackGroup()
        super().__init__(node, PickOrPlace, server_ns,
                         execute_callback=self._execute_cb,
                         callback_group=self._server_cbg,
                         group_field='robot_name')

    def _execute_cb(self, goal_handle):
        request = goal_handle.request
        self.logger.info('### Do %s ###'
                         % 'picking' if request.pick else 'placing')
        node    = self.node
        com     = node.com
        gripper = node.gripper(request.robot_name)
        if request.pick:
            object_id = self._get_object_id(request.pose.header.frame_id)
            eef_link  = ''
        else:
            object_id = self._get_object_id(request.subframe_link)
            eef_link  = request.subframe_link

        try:
            # [1] Move stage: Go to approach pose.
            self.enter_stage(goal_handle, 'move')
            speed   = request.speed_fast if request.pick else \
                      request.speed_slow
            success = node.go_to_pose_goal(request.robot_name, request.pose,
                                           request.approach_offset, speed,
                                           end_effector_link=eef_link)
            if not success:
                raise self._Error(PickOrPlace.Result.MOVE_FAILURE,
                                  'move', 'Failed to go to approach pose')

            # [2] Approach stage: Go to pick/place pose.
            self.enter_stage(goal_handle, 'approach', 'move')
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
            if not success:
                if request.pick:
                    gripper.release()
                raise self._Error(PickOrPlace.Result.APPROACH_FAILURE,
                                  'approach', 'Failed to approach target')

            # [3] Grasp/release stage: Grasp or release at pick/place pose.
            self.enter_stage(goal_handle, 'grasp/release', 'approach')
            if request.pick:
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
                                      self._get_object_id(gripper.tip_link))

            # [4] Depart stage: Go back to departure(pick) or approach(place)
            #     pose.
            self.enter_stage(goal_handle, 'depart', 'grasp/release')
            if request.pick:
                gripper.postgrasp()                 # Postgrasp (not wait)
                speed  = request.speed_slow
                pose   = request.pose
                offset = request.departure_offset
            else:
                speed = request.speed_fast
                if object_id != '':
                    pose = PoseStamped(request.pose.header,
                                       self._concatenate_poses(
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
            if not success:
                if request.pick:
                    gripper.release()
                    if object_id != '':
                        com.detach_object(object_id,
                                          original_object_info.parent_link,
                                          self._get_object_id(
                                              gripper.tip_link))
                raise self._Error(PickOrPlace.Result.APPROACH_FAILURE,
                                  'depart', 'Failed to depart from target')

            # [5] Verify stage: Verify success of postgrasp.
            self.enter_stage(goal_handle, 'verify', 'depart')
            if request.pick and \
               not gripper.grasped():  # Wait for postgrasp completed
                gripper.release()
                if object_id != '':
                    com.detach_object(object_id,
                                      original_object_info.parent_link,
                                      self._get_object_id(gripper.tip_link))
                    com.move_object(object_id, original_object_info.pose,
                                    self._get_subframe(
                                        request.pose.header.frame_id))
                raise self._Error(PickOrPlace.Result.GRASP_FAILURE,
                                  'verify', 'Failed to grasp')

            # [Final] Goal succeeded.
            goal_handle.succeed()
            self.logger.info('### %s succeeded. ###',
                             'Pick' if request.pick else 'Place')
            return PickOrPlace.Result(result=PickOrPlace.Result.SUCCESS,
                                      stage='')

        except self._Error as err:
            self.logger.error(err)
            goal_handle.abort()
            return PickOrPlace.Result(stage=err.stage, result=err.result)

        finally:
            if object_id != '':
                #com.disallow_collision(object_id, gripper.tip_link)
                com.reset_touch_links()

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

#************************************************************************
#  class PickOrPlaceTask                                                *
#************************************************************************
class PickOrPlaceTask(PickOrPlaceTaskClient):
    def __init__(self, node, server_ns='pick_or_place'):
        self._server = PickOrPlaceTaskServer(node, server_ns)
        super().__init__(node, server_ns)
