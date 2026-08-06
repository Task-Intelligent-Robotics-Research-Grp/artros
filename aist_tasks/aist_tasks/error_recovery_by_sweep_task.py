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
from rclpy.node                  import Node
from rclpy.callback_groups       import MutuallyExclusiveCallbackGroup
from task_wrappers.action_server import ActionServer
from task_wrappers.action_client import GroupedSimpleActionClient
from aist_msgs.action            import Sweep
from .request_help_task          import RequestHelpTask
from .sweep_task                 import SweepTask

#*********************************************************************
#  class ErrorRecoveryBySweepTaskClient                              *
#*********************************************************************
class ErrorRecoveryBySweepTaskClient(GroupedSimpleActionClient):
    def __init__(self, node: Node, server_ns: str='error_recovery_by_sweep'):
        super().__init__(node, ErrorRecoveryBySweep, server_ns,
                         callback_group=MutuallyExclusiveCallbackGroup(),
                         group_field='robot_name')
        self.wait_for_server()

    def send_goal(self, robot_name, pose, part_id, message,
                  *, timeout_sec=0.0):
        return super().send_goal(
                  ErrorRecoveryBySweep.Goal(robot_name=robot_name,
                                            item_id=part_id,
                                            pose=pose,
                                            message=message),
                  feedback_callback=self.stage_feedback_cb,
                  timeout_sec=timeout_sec)

#*********************************************************************
#  class ErrorRecoveryBySweepTaskServer                              *
#*********************************************************************
class ErrorRecoveryBySweepTaskServer(ActionServer):
    def __init__(self, node, server_ns='error_recovery_by_sweep'):
        super().__init__(node, ErrorRecoveryBySweep, server_ns,
                         self._execute_cb,
                         callback_group=MutuallyExclusiveCallbackGroup(),
                         group_field='robot_name')

        self._request_help = RequestHelpTask(self)
        self._sweep        = SweepTask(self)

    def _execute_cb(self, goal_handle):
        self.logger.info("*** Do sweeping ***")

        request = goal_handle.request
        node    = self.node

        # [1] Sweep ready stage: Go to approach pose.
        stage = self.enter_stage(goal_handle, 'sweep_ready')
        success = self.go_to_named_pose(robot_name, 'sweep_ready')

        # [1] Move stage: Go to approach pose.
        success = node.go_to_pose_goal(request.robot_name, request.pose,
                                       request.approach_offset,
                                       request.speed_fast)
        if not success:
            raise ActionServer._Error('Failed to go to approach pose',
                                      stage=stage)

        # [2] Approach stage: Go to sweep pose.
        stage   = self.enter_stage(goal_handle, 'approach', stage)
        success = node.go_to_pose_goal(request.robot_name, request.pose,
                                       request.sweep_offset,
                                       request.speed_slow)
        if not success:
            raise ActionServer._Error('Failed to go to sweep pose',
                                      stage=stage)

        # [3] Sweep stage: Sweep the object.
        stage  = self.enter_stage(goal_handle, 'sweep', stage)
        offset = list(request.sweep_offset)
        offset[1] += request.sweep_length
        success = node.go_to_pose_goal(request.robot_name, request.pose,
                                       offset, request.speed_fast)
        if not success:
            raise ActionServer._Error('Failed to go to sweep', stage=stage)

        # [4] Depart stage: Go back to departure pose.
        stage   = self.enter_stage(goal_handle, 'depart', stage)
        success = node.go_to_pose_goal(request.robot_name, request.pose,
                                       request.departure_offset,
                                       request.speed_fast)
        if not success:
            raise ActionServer._Error('Failed to go to departure pose',
                                      stage=stage)

        # [Final] Goal succeeded.
        goal_handle.succeed()
        self.logger.info('*** Sweep succeeded. ***')
        return Sweep.Result(stage=stage)

#************************************************************************
#  class ErrorRecoveryBySweepTask                                       *
#************************************************************************
class ErrorRecoveryBySweepTask(SweepTaskClient):
    def __init__(self, node, server_ns='error_recovery_by_sweep'):
        self._server = ErrorRecoveryBySweepTaskServer(node, server_ns)
        super().__init__(node, server_ns)
