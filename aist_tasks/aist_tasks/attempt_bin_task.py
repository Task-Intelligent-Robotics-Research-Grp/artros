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
from rclpy.node                    import Node
from rclpy.action                  import GoalResponse, CancelResponse
from rclpy.callback_groups         import MutuallyExclusiveCallbackGroup
from action_msgs.msg               import GoalStatus
from geometry_msgs.msg             import (PoseStamped, QuaternionStamped,
                                           Transform, Vector3, Quaternion)
from aist_msgs.action              import PickOrPlace, AttemptBin
from aist_tasks.pick_or_place_task import PickOrPlaceTaskClient
from aist_graspability.client      import GraspabilityClient
from task_wrappers.action_client   import GroupedSimpleActionClient
from task_wrappers.action_server   import ActionServer
from aist_utility.geometry_msgs    import format_pose

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
        super().__init__(node, AttemptBin, server_ns, self._execute_cb,
                         callback_group=MutuallyExclusiveCallbackGroup(),
                         group_field='robot_name')

    def _execute_cb(self, goal_handle):
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
        request = goal_handle.request

        # [1] Prepare stage: Get properties of bin and part.
        stage = self.enter_stage(goal_handle, 'prepare')
        bin_props = self.node.bin_props.get(request.bin_id)
        if not bin_props:
            raise ActionServer._Error('unknown bin_id[%s]' % request.bin_id,
                                      stage=stage)
        part_id    = bin_props['part_id']
        part_props = self.node.part_props.get(part_id)
        if not part_props:
            raise ActionServer._Error('unknown part_id[%s]' % part_id,
                                      stage=stage)

        if self._is_eye_on_hand(request.robot_name, part_props['camera_name']):
            # [2] Move camera stage: Go to pose for capturing bin.
            #   Move to 0.15m above the bin if the camera is mounted
            #   on the robot.
            stage = self.enter_stage(goal_handle, 'move_camera', stage)
            success = self.node.go_to_frame(request.robot_name,
                                            bin_props['name'], (0, 0, 0.15))
            if not success:
                raise ActionServer._Error('failed to move camera', stage=stage)

        if pick_poses is None:
            # [3] Search stage: Search for graspabilities.
            stage = self.enter_stage(goal_handle, 'search', stage)
            status, result = self.node.search_bin(request.bin_id)
            if status != GoalStatus.STATUS_SUCCEEDED:
                raise ActionServer._Error('failed to search graspabilities',
                                          stage=stage)
            pick_poses = result.graspabilities.poses

        # Attempt to pick the item.
        nattempts = 0
        for p in pick_poses.poses:
            if nattempts == request.max_attempts:
                break

            pose = PoseStamped(header=pick_poses.header, pose=p)
            if self._is_close_to_fail_poses(pose):
                continue

            self.logger.warn('### pose=%s' % format_pose(pose))

            # Perform picking.
            stage = self.enter_stage(goal_handle, 'pick', stage)
            status, result = self.node.pick(request.robot_name, part_id, pose)

            # A. Pick succeeded.
            if status == GoalStatus.STATUS_SUCCEEDED:
                # if self._do_error_recovery and \
                #    self.node.using_hmi_graspability_params:
                #     self.node.restore_original_graspability_params(bin_id)

                # [4] Place stage: Begin placing and wait until reaching
                #   approach pose.
                stage = self.enter_stage(goal_handle, 'place', stage)
                self.node.place_at_frame(request.robot_name, part_id,
                                         part_props['destination'],
                                         offset=(0.0, place_offset, 0.0),
                                         timeout_sec=0.0)
                self.node.pick_or_place_wait(target_stage='approach')

                # [5] Search stage: Search graspabilities for the next try.
                stage = self.enter_stage(goal_handle, 'search', stage)
                poses = self.node.search_bin(bin_id).poses

                # [6] Wait until placing finished.
                status, result = self.node.pick_or_place_wait_for_result()
                return status == GoalStatus.STATUS_SUCCEEDED, poses

            # B. Pick failed.
            elif status == GoalStatus.STATUS_ABORTED:
                # B-1. Pick failed due to error in moving to approach/pick pose
                if result.stage in ('move', 'approach'):
                    self._fail_poses.append(pose)

                # B-2. Pick failed due to error in departing from pick pose
                elif result.stage == 'depart':
                    raise ActionServer._Error('failed to depart from pick pose',
                                              stage=stage)

                # B-3. Pick failed due to error in grasping
                elif result.stage == 'verify':
                    # if self._do_error_recovery and \
                    #    self.node.using_hmi_graspability_params and \
                    #    self._do_error_recovery(robot_name, pose, part_id):
                    #     self.node.restore_original_graspability_params(bin_id)
                    #     return True, None
                    # else:
                    #     self._fail_poses.append(pose)
                    #     nattempts += 1
                    self._fail_poses.append(pose)
                    nattempts += 1

            # C. Pick canceled.
            elif status == GoalStatus.STATUS_CANCELED:
                raise ActionServer._Preempted(stage)

        # Here, no graspability poses remained or max_attempts attained.
        # if self._do_error_recovery:
        #     if self.node.using_hmi_graspability_params:
        #         self.node.restore_original_graspability_params(bin_id)
        #         return False, None
        #     else:
        #         self.node.set_hmi_graspability_params(bin_id)
        #         return True, None
        # else:
        #     return False, None
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

#************************************************************************
#  class AttemptBinTask                                                 *
#************************************************************************
class AttemptBinTask(AttemptBinTaskClient):
    def __init__(self, node, server_ns='attempt_bin'):
        self._server = AttemptBinTaskServer(node, server_ns)
        super().__init__(node, server_ns)
