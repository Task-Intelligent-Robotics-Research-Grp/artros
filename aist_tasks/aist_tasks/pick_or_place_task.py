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
import numpy as np
import tf_transformations as tfs

from rclpy.action                import GoalResponse, CancelResponse
from rclpy.callback_groups       import MutuallyExclusiveCallbackGroup
from action_msgs.msg             import GoalStatus
from aist_msgs.action            import PickOrPlace
from geometry_msgs.msg           import Point, Quaternion, Pose, PoseStamped
from task_wrappers.action_server import ActionServer
from task_wrappers.action_client import GroupedSimpleActionClient

#************************************************************************
#  local functions                                                      *
#************************************************************************
def _decompose_link_name(link_name):
    """ Decompose the given link name into object_id and subframe.

    Args:
      link_name: name of the link

    Returns:
      A tuple of object ID and subframe name of the collision object,
      if `link_name` is a fullname of subframe of any collision object.
      A tuple ('', `link_name`), otherwise.

    Examples:
      * 'panel_bearing/base_link` => ('panel_bearing', 'base_link')
      * 'a_bot_gripper_tip_link'  => ('', 'a_bot_gripper_tip_link')
    """
    tokens = link_name.rsplit('/', 1)
    return tokens if len(tokens) == 2 else ('', link_name)

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
    t = tfs.translation_from_matrix(T)
    q = tfs.quaternion_from_matrix(T)
    return Pose(position=Point(x=t[0], y=t[1], z=t[2]),
                orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))

#************************************************************************
#  class PickOrPlaceTaskClient                                          *
#************************************************************************
class PickOrPlaceTaskClient(GroupedSimpleActionClient):
    Success = (GoalStatus.STATUS_SUCCEEDED, PickOrPlace.Result(stage=''))

    def __init__(self, node, server_ns='pick_or_place'):
        super().__init__(node, PickOrPlace, server_ns,
                         callback_group=MutuallyExclusiveCallbackGroup(),
                         group_field='robot_name')
        self.wait_for_server()

    def send_goal(self, robot_name, pick, pose, offset,
                  approach_offset, departure_offset, speed_fast, speed_slow,
                  *, subframe_link='', timeout_sec=0.0):
        return super().send_goal(PickOrPlace.Goal(
                                     robot_name=robot_name,
                                     pick=pick,
                                     subframe_link=subframe_link,
                                     pose=pose,
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
    def __init__(self, node, server_ns='pick_or_place'):
        super().__init__(node, PickOrPlace, server_ns, self._execute_cb,
                         callback_group=MutuallyExclusiveCallbackGroup(),
                         group_field='robot_name')

    def _execute_cb(self, goal_handle):
        request = goal_handle.request
        self.logger.info('### %s ###' % ('Pick' if request.pick else 'Place'))
        node    = self.node
        com     = node.com
        gripper = node.gripper(request.robot_name)
        if request.pick:
            object_id = _decompose_link_name(request.pose.header.frame_id)[0]
            eef_link  = ''
        else:
            object_id = _decompose_link_name(request.subframe_link)[0]
            eef_link  = request.subframe_link

        try:
            # [1] Move stage: Go to approach pose.
            stage   = self.enter_stage(goal_handle, 'move')
            speed   = request.speed_fast if request.pick else \
                      request.speed_slow
            success = node.go_to_pose_goal(request.robot_name, request.pose,
                                           request.approach_offset, speed,
                                           end_effector_link=eef_link)
            if not success:
                raise ActionServer._Error('Failed to go to approach pose',
                                          stage=stage)

            # [2] Approach stage: Go to pick/place pose.
            stage = self.enter_stage(goal_handle, 'approach', stage)
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
                raise ActionServer._Error('Failed to approach target',
                                          stage=stage)

            # [3] Grasp/release stage: Grasp or release at pick/place pose.
            stage = self.enter_stage(goal_handle, 'grasp/release', stage)
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
                                      _decompose_link_name(
                                          gripper.tip_link)[0])

            # [4] Depart stage: Go back to departure(pick) or approach(place)
            #     pose.
            stage = self.enter_stage(goal_handle, 'depart', stage)
            if request.pick:
                gripper.postgrasp()                 # Postgrasp (not wait)
                speed  = request.speed_slow
                pose   = request.pose
                offset = request.departure_offset
            else:
                speed = request.speed_fast
                if object_id != '':
                    pose = PoseStamped(header=request.pose.header,
                                       pose=_concatenate_poses(
                                                request.pose.pose,
                                                node.pose_from_xyzrpy(
                                                    request.departure_offset) \
                                                .pose,
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
                                          _decompose_link_name(
                                              gripper.tip_link)[0])
                        self.logger.warn('### Detach %s' % object_id)
                raise ActionServer._Error('Failed to depart from target',
                                          stage=stage)

            # [5] Verify stage: Verify success of postgrasp.
            stage = self.enter_stage(goal_handle, 'verify', stage)
            if request.pick and \
               not node.get_parameter('use_sim_time') \
                       .get_parameter_value().bool_value and \
               not gripper.grasped():
                gripper.release()
                if object_id != '':
                    com.detach_object(object_id,
                                      original_object_info.parent_link,
                                      _decompose_link_name(
                                          gripper.tip_link)[0])
                    com.move_object(object_id, original_object_info.pose,
                                    _decompose_link_name(
                                        request.pose.header.frame_id)[1])
                raise ActionServer._Error('Failed to grasp', stage=stage)

            # [Final] Goal succeeded.
            goal_handle.succeed()
            self.logger.info('### %s succeeded. ###'
                             % ('Pick' if request.pick else 'Place'))
            return PickOrPlace.Result(stage=stage)

        finally:
            if object_id != '':
                #com.disallow_collision(object_id, gripper.tip_link)
                com.reset_touch_links()
                self.logger.info('reset touch links')

#************************************************************************
#  class PickOrPlaceTask                                                *
#************************************************************************
class PickOrPlaceTask(PickOrPlaceTaskClient):
    def __init__(self, node, server_ns='pick_or_place'):
        self._server = PickOrPlaceTaskServer(node, server_ns)
        super().__init__(node, server_ns)
