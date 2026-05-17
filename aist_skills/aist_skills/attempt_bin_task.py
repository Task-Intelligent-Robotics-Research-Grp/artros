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
from rclpy.node                     import node
from math                           import radians
from geometry_msgs.msg              import (PoseStamped, QuaternionStamped,
                                            Transform, Vector3, Quaternion)
from action_msgs.msg                import GoalStatus
from aist_msgs.action               import PickOrPlace, AttemptBin
from task_wrappers.task_client      import GroupedSimpleTaskClient
from task_wrappers.task_server      import TaskServer,
from aist_skills.pick_or_place_task import PickOrPlaceTaskClient
from aist_graspability.client       import GraspabilityClient

#*********************************************************************
#  class AttemptBinTaskClient                                        *
#*********************************************************************
class AttemptBinTaskClient(GroupedSimpleTaskClient):
    def __init__(self, node: Node, server_ns: str='attempt_bin'):
        super().__init__(node, AttemptBin, server_ns, group_field='robot_name')
        self.wait_for_server()

    def send_goal(self, bin_id, pick_all, max_attempts):
        super().send_goal(AttemptBinGoal(bin_id=bin_id, pick_all=pick_all,
                                         max_attempts=max_attempts))

#*********************************************************************
#  class AttemptBinTaskServer                                        *
#*********************************************************************
class AttemptBinTaskServer(TaskServer):
    def __init__(self, node: Node, server_ns: str='attempt_bin',
                 kitting_params,
                 do_error_recovery=None, cancel_error_recovery=None):
        super().__init__(node, AttemptBin, server_ns, self._execute_cb,
                         group_field='robot_name')
        self._graspability   = GraspabilityClient(node)
        self._pick_or_place  = PickOrPlaceTaskClient(node)
        self._kitting_params = picking_params

        self._graspability.load_borders()

    def _execute_cb(self, goal_handle):
        try_next     = True
        pick_poses   = None
        place_offset = 0.020
        self._clear_fail_poses()
        while try_next:
            try_next, pick_poses \
                = self._attempt_bin(goal_handle, pick_poses, place_offset)
            if not goal_handle.is_active:
                return AttemptBin.Result()
            if not goal_handle.request.pick_all:
                break
            place_offset = -place_offset
        goal_handle.succeed()
        self.logger.info('(AttemptBinTask) SUCCEEDED')
        return AttemptBin.Result()

    def _attempt_bin(self, goal_handle, poses, place_offset):
        bin_props = self._kitting_params['bin_props'] \
                        .get(goal_handle.request.bin_id)
        if not bin_props:
            goal_handle.abort()
            self.logger.error('unknown bin_id[%s]'
                              % goal_handle.request.bin_id)
            return False, None  # (no parts remained, no graspabilities)
        part_props = self._kitting_params['part_props'] \
                         .get(bin_props['part_id'])
        if not part_props:
            goal_handle.abort()
            self.logger.error('unknown part_id[%s]' % bin_props['part_id'])
            return False, None  # (no parts remained, no graspabilities)

        # Move to 0.15m above the bin if the camera is mounted on the robot.
        if self._is_eye_on_hand(robot_name, part_props['camera_name']):
            self.node.go_to_frame(robot_name, bin_props['name'], (0, 0, 0.15))

        # Search for graspabilities.
        if pick_poses is None:
            status, result = self.node.search_bin(goal_handle.request.bin_id)
            if status != GoalStatus.STATUS_SUCCEEDED:
                self.logger.error('failed to search graspabilities')
                return False, None
            pick_poses = result.poses

        if not goal_handle.is_active:
            return False, None  # (no parts remained, no graspabilities)

        # Attempt to pick the item.
        nattempts = 0
        for p in pick_poses.poses:
            if nattempts == max_attempts:
                break

            pose = PoseStamped(pick_poses.header, p)
            if self._is_close_to_fail_poses(pose):
                continue

            # Perform picking.
            pick_result = self.node.pick(robot_name, pose, part_id)
            if not self._server.is_active():
                return False, None

            # 1. Pick succeeded
            if pick_result == PickOrPlaceResult.SUCCESS:
                if self._do_error_recovery and \
                   self.node.using_hmi_graspability_params:
                    self.node.restore_original_graspability_params(bin_id)

                # Begin placing and wait until reaching approach pose.
                self.node.place_at_frame(robot_name, part_props['destination'],
                                        part_id,
                                        offset=(0.0, place_offset, 0.0),
                                        wait=False)
                self.node.pick_or_place_wait_for_stage(
                    PickOrPlaceFeedback.APPROACHING)

                # Search graspabilities for the next try.
                poses = self.node.search_bin(bin_id).poses

                # Wait until placing finished.
                place_result = self.node.pick_or_place_wait_for_result()
                return place_result == PickOrPlaceResult.SUCCESS, poses

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
