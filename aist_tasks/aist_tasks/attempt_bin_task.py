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
from rclpy.node                     import Node
from rclpy.action                   import GoalResponse, CancelResponse
from rclpy.callback_groups          import MutuallyExclusiveCallbackGroup
from action_msgs.msg                import GoalStatus
from geometry_msgs.msg              import (PoseStamped, QuaternionStamped,
                                            Transform, Vector3, Quaternion)
from aist_msgs.action               import PickOrPlace, AttemptBin
from aist_skills.pick_or_place_task import PickOrPlaceTaskClient
from aist_graspability.client       import GraspabilityClient
from task_wrappers.task_client      import GroupedSimpleActionClient
from task_wrappers.task_server      import ActionServer

#*********************************************************************
#  class AttemptBinTaskClient                                        *
#*********************************************************************
class AttemptBinTaskClient(GroupedSimpleActionClient):
    Success = (GoalStatus.STATUS_SUCCEEDED, AttemptBin.Result(stage=''))

    def __init__(self, node: Node, server_ns: str='attempt_bin'):
        super().__init__(node, AttemptBin, server_ns,
                         callback_group=MutuallyExclusiveCallbackGroup(),
                         group_field='robot_name')
        self.wait_for_server()

    def send_goal(self, robot_name, bin_id, pick_all, max_attempts,
                  *, timeout_sec=0.0):
        return super().send_goal(AttemptBin.Goal(robot_name=robot_name,
                                                 bin_id=bin_id,
                                                 pick_all=pick_all,
                                                 max_attempts=max_attempts),
                                 feedback_callback=self.stage_feedback_cb,
                                 timeout_sec=timeout_sec)

#*********************************************************************
#  class AttemptBinTaskServer                                        *
#*********************************************************************
class AttemptBinTaskServer(ActionServer):
    def __init__(self, node: Node, server_ns: str='attempt_bin'):
        super().__init__(node, AttemptBin, server_ns, self._user_execute_cb,
                         callback_group=MutuallyExclusiveCallbackGroup(),
                         group_field='robot_name')
        self._graspability.load_borders()

    def _user_execute_cb(self, goal_handle):
        try_next     = True
        pick_poses   = None
        place_offset = 0.020
        self._clear_fail_poses()

        while try_next:
            try:
                try_next, pick_poses \
                    = self._attempt_bin(goal_handle, pick_poses, place_offset)
                if not goal_handle.is_active:
                    return AttemptBin.Result()
                if not goal_handle.request.pick_all:
                    break
                place_offset = -place_offset
            finally:
                pass
        goal_handle.succeed()
        self.logger.info('(AttemptBinTask) SUCCEEDED')
        return AttemptBin.Result(stage='')

    def _attempt_bin(self, goal_handle, pick_poses, place_offset):
        bin_props = self.node.bin_props.get(goal_handle.request.bin_id)
        if not bin_props:
            raise self._Error('unknown bin_id[%s]'
                              % goal_handle.request.bin_id,
                              stage='')
        part_props = self.node.part_props.get(bin_props['part_id'])
        if not part_props:
            raise self._Error('unknown part_id[%s]' % bin_props['part_id'],
                              stage='')

        if self._is_eye_on_hand(robot_name, part_props['camera_name']):
            # [1] Move camera stage: Go to pose for capturing bin.
            #   Move to 0.15m above the bin if the camera is mounted
            #   on the robot.
            stage = self.enter_stage(goal_handle, 'move_camera')
            self.node.go_to_frame(robot_name, bin_props['name'], (0, 0, 0.15))

        if pick_poses is None:
            # [2] Search stage: Search for graspabilities.
            stage = self.enter_stage(goal_handle, 'search', stage)
            status, result = self.node.search_bin(goal_handle.request.bin_id)
            if status != GoalStatus.STATUS_SUCCEEDED:
                raise self._Error('failed to search graspabilities',
                                  stage='search')
            pick_poses = result.poses

        # Attempt to pick the item.
        nattempts = 0
        for p in pick_poses.poses:
            if nattempts == max_attempts:
                break

            pose = PoseStamped(header=pick_poses.header, pose=p)
            if self._is_close_to_fail_poses(pose):
                continue

            # Perform picking.
            self.enter_stage(goal_handle, 'pick', 'search')
            status, result = self.node.pick(robot_name, pose, part_id)

            # 1. Pick succeeded
            if status == GoalStatus.STATUS_SUCCEEDED:
                if self._do_error_recovery and \
                   self.node.using_hmi_graspability_params:
                    self.node.restore_original_graspability_params(bin_id)

                # [3] Place stage: Begin placing and wait until reaching
                #   approach pose.
                self.enter_stage(goal_handle, 'place', 'pick')
                self.node.place_at_frame(robot_name, part_props['destination'],
                                         part_id,
                                         offset=(0.0, place_offset, 0.0),
                                         timeout_sec=0.0)
                self.node.pick_or_place_wait(target_stage='approach')

                # [4] Search stage: Search graspabilities for the next try.
                self.enter_stage(goal_handle, 'search', 'place')
                poses = self.node.search_bin(bin_id).poses

                # [4] Wait until placing finished.
                status, result = self.node.pick_or_place_wait_for_result()
                return status == GoalStatus.STATUS_SUCCEEDED, poses

            # 2. Pick failed due to error in moving to approach/pick pose
            elif pick_result in (PickOrPlaceResult.MOVE_FAILURE,
                                 PickOrPlaceResult.APPROACH_FAILURE):
                self._fail_poses.append(pose)

            # 3. Pick failed due to error in departing from pick pose
            elif pick_result == PickOrPlaceResult.DEPARTURE_FAILURE:
                self._server.set_aborted()
                self.logger.err('(AttemptBin) Failed to depart from pick/place pose')
                return False, None

            # 4. Pick failed due to error in grasping
            elif pick_result == PickOrPlaceResult.GRASP_FAILURE:
                if self._do_error_recovery and \
                   self.node.using_hmi_graspability_params and \
                   self._do_error_recovery(robot_name, pose, part_id):
                    self.node.restore_original_graspability_params(bin_id)
                    return True, None
                else:
                    self._fail_poses.append(pose)
                    nattempts += 1

        # Here, no graspability poses remained or max_attempts attained.
        if self._do_error_recovery:
            if self.node.using_hmi_graspability_params:
                self.node.restore_original_graspability_params(bin_id)
                return False, None
            else:
                self.node.set_hmi_graspability_params(bin_id)
                return True, None
        else:
            return False, None

    def _preempt_cb(self):
        self._self.node.pick_or_place_cancel_goal()
        if self._cancel_error_recovery:
            self._cancel_error_recovery()
            self._self.node.restore_original_graspability_params(
                self._server.current_goal.get_goal().bin_id)
        self._server.set_preempted()
        self.logger.warn('(AttemptBin) CANCELLED')

    # Utilities
    def _clear_fail_poses(self):
        self._fail_poses = []

    def _is_eye_on_hand(self, robot_name, camera_name):
        return camera_name == robot_name + '_camera'

    def _is_close_to_fail_poses(self, pose):
        for fail_pose in self._fail_poses:
            if self._is_close_to_fail_pose(pose, fail_pose, 0.005):
                return True
        return False

    def _is_close_to_fail_pose(self, pose, fail_pose, tolerance):
        position      = pose.pose.position
        fail_position = fail_pose.pose.position
        if abs(position.x - fail_position.x) > tolerance or \
           abs(position.y - fail_position.y) > tolerance or \
           abs(position.z - fail_position.z) > tolerance:
            return False
        return True
