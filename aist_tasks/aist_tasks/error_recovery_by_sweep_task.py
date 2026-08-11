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
from aist_msgs.action            import ErrorRecoveryBySweep
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
        def _compute_sweep_dir(self, pose: PoseStamped, pointing: Pointing):
            ppos = pose.pose.position
            fpos = self.transform_points_to_target_frame(pointing.header,
                                                         [pointing.point],
                                                         pose.header.frame_id)\
                  .point
            sdir = (fpos.x - ppos.x, fpos.y - ppos.y, fpos.z - ppos.z)
            return tuple(sdir / np.linalg.norm(sdir))

        def _fix_orientation(self, orientation: Quaternion, sweep_dir):
            R = tfs.quaternion_matrix((orientation.x, orientation.y,
                                       orientation.z, orientation.w))
            nz = R[0:3, 2]
            ny = sweep_dir - nz * np.dot(nz, sweep_dir)
            R[0:3, 1] = ny/np.linalg.norm(ny)
            R[0:3, 0] = np.cross(R[0:3, 1], nz)
            q = tfs.quaternion_from_matrix(R)
            return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        self.logger.info("=== ErrorRecoveryBySweep ===")

        request = goal_handle.request.request
        node    = self.node
        stop    = lambda: node.stop(request.robot_name)

        # [1] Sweep ready stage: Go to sweep ready pose.
        with ActionServer.Stage(self, goal_handle, 'sweep_ready',
                                stop) as stage:
            success = node.go_to_named_pose(request.robot_name, 'sweep_ready')
            if not success:
                raise ActionServer.Error('Failed to go to sweep ready pose',
                                         stage=stage.name)

        # [2] Request help stage:
        with ActionServer.Stage(self, goal_handle, 'request_help',
                                stop) as stage:
            message = 'Picking_failed!'
            status, result = self._request_help.send_goal(request.robot_name,
                                                          request.pose,
                                                          request.item_id,
                                                          message)
            if status == GoalStatus.STATUS_ABORTED:
                raise ActionServer.Error('Failed to get sweep direction',
                                          stage=stage.name)
            sweep_dir = self._compute_sweep_dir(request.pose, result.pointing)

        # [3] Sweep stage:
        with ActionServer.Stage(self, goal_handle, 'sweep') as stage:
            pose = copy.deepcopy(request.pose)
            pose.pose.orientation = _fix_orientation(
                                        request.pose.pose.orientation,
                                        sweep_dir)
            params = self.sweep_parameters[part_id]
            status, result = self._sweep.send_goal(robot_name, pose,
                                                   params['sweep_length'],
                                                   params['sweep_offset'],
                                                   params['approach_offset'],
                                                   params['departure_offset'],
                                                   params['speed_fast'],
                                                   params['speed_slow'])
            if status == GoalStatus.STATUS_ABORTED:
                raise ActionServer.Error('Failed to sweep', stage=stage.name)

        # [Final] Final stage: Goal succeeded.
        goal_handle.succeed()
        self.logger.info('=== ErrorRecoveryBySweep succeeded ===')
        return ErrorRecoveryBySweep.Result(stage='')


#************************************************************************
#  class ErrorRecoveryBySweepTask                                       *
#************************************************************************
class ErrorRecoveryBySweepTask(SweepTaskClient):
    def __init__(self, node, server_ns='error_recovery_by_sweep'):
        self._server = ErrorRecoveryBySweepTaskServer(node, server_ns)
        super().__init__(node, server_ns)
