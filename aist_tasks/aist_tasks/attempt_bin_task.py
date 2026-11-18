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
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from action_msgs.msg       import GoalStatus
from geometry_msgs.msg     import PoseStamped
from aist_msgs.action      import AttemptBin
from task_wrappers         import GroupedSimpleActionClient, ActionServer

from rclpy.node            import Node
from rclpy.action.server   import ServerGoalHandle
from geometry_msgs.msg     import Point, PoseArray
from typing                import Optional, List

#*********************************************************************
#  class AttemptBinTaskClient                                        *
#*********************************************************************
class AttemptBinTaskClient(GroupedSimpleActionClient):
    def __init__(self, node: Node, server_ns: str='attempt_bin'):
        super().__init__(node, AttemptBin, server_ns,
                         callback_group=MutuallyExclusiveCallbackGroup(),
                         group_field='robot_name')

    def send_goal(self, robot_name: str, bin_id: str, pick_all: bool,
                  max_attempts: int, *, timeout_sec: Optional[float]=0.0):
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

    def _execute_cb(self, goal_handle: ServerGoalHandle):
        def _is_eye_on_hand(robot_name: str, camera_name: str):
            return camera_name == robot_name + '_camera'

        goal                      = goal_handle.request
        node                      = self.node
        stop                      = lambda: node.stop(goal.robot_name)
        cancel_pick_or_place      = lambda: node.pick_or_place_task \
                                                .cancel_goal(goal.robot_name)
        cancel_pick_or_place_tool = lambda: node.pick_or_place_tool_task \
                                                .cancel_goal(goal.robot_name)

        bin_props = node.bin_props.get(goal.bin_id)
        if not bin_props:
            raise ActionServer.Error('unknown bin_id[%s]' % goal.bin_id,
                                     stage='')
        part_id    = bin_props['part_id']
        part_props = node.part_props.get(part_id)
        if not part_props:
            raise ActionServer.Error('unknown part_id[%s]' % part_id,
                                     stage='')

        # [1] 'pick_tool' stage: Pick tool requested for the part.
        with ActionServer.Stage(self, goal_handle, 'pick_tool',
                                cancel_pick_or_place_tool) as stage:
            status, result = node.pick_tool(goal.robot_name,
                                            part_props['gripper_name'])
            if status is GoalStatus.STATUS_ABORTED:
                raise ActionServer.Error('Failed to pick tool!',
                                         stage=stage.extend_name(
                                             result.stage))

        # [2] 'go_home' stage: Go to home pose.
        with ActionServer.Stage(self, goal_handle, 'go_home', stop) as stage:
            success = node.go_to_named_pose(goal.robot_name, 'home')
            if not success:
                raise ActionServer.Error('Failed to go home', stage=stage.name)

        use_fine_parameters = False  # Use default graspability parameters
        pick_poses          = None
        fail_poses          = []
        place_offset        = 0.020

        while True:
            # If no graspability poses available, search for them.
            if not pick_poses:
                if _is_eye_on_hand(goal.robot_name, part_props['camera_name']):
                    # [3] 'move_camera' stage: Go to pose for capturing bin.
                    #     Move to 0.15m above the bin if the camera is mounted
                    #     on the robot.
                    with ActionServer.Stage(self, goal_handle, 'move_camera',
                                            stop) as stage:
                        success = node.go_to_frame(goal.robot_name,
                                                   bin_props['name'],
                                                   (0, 0, 0.15))
                        if not success:
                            raise ActionServer.Error('Failed to move camera',
                                                     stage=stage.name)

                # [4] 'search' stage: Search for graspabilities.
                with ActionServer.Stage(self, goal_handle, 'search') as stage:
                    status, result = node.search_bin(goal.bin_id,
                                                     use_fine_parameters)
                    if status is GoalStatus.STATUS_ABORTED:
                        raise ActionServer.Error(
                            'Failed to search graspabilities',
                            stage=stage.name)
                    pick_poses = result.graspabilities.poses
                    fail_poses = []

            # [5] 'pick' stage: Pick a part at a pose selected from pick_poses.
            with ActionServer.Stage(self, goal_handle, 'pick',
                                    cancel_pick_or_place) as stage:
                # Attempt only once if fine graspability parameters are used.
                status, result, pose = self._attempt_pick(
                                           goal, pick_poses, fail_poses,
                                           1 if use_fine_parameters else \
                                           goal.max_attempts)
                if use_fine_parameters and status is GoalStatus.STATUS_ABORTED:
                    self.logger.warn('--- AttemptBin: aborted at stage[%s] under fine parameters'
                                     % stage.name)
                    raise ActionServer.Error('Failed to pick',
                                             stage=stage.extend_name(
                                                 result.stage),
                                             pose=pose)

            if status is GoalStatus.STATUS_SUCCEEDED:  # pick succeeded
                # [6] 'place' stage: Begin placing and wait until reaching
                #     approach pose.
                with ActionServer.Stage(self, goal_handle, 'place',
                                        cancel_pick_or_place) as stage:
                    # Place the picked part (not wait).
                    node.place_at_frame(goal.robot_name, part_id,
                                        part_props['destination'],
                                        offset=(0.0, place_offset, 0.0),
                                        timeout_sec=0.0)
                    place_offset        = -place_offset
                    use_fine_parameters = False
                    fail_poses          = []

                    if _is_eye_on_hand(goal.robot_name,
                                       part_props['camera_name']):
                        # Wait until placing finished.
                        status, result = node.pick_or_place_task \
                                             .wait(goal.robot_name)
                        if status is GoalStatus.STATUS_ABORTED:
                            raise ActionServer.Error('Failed to place',
                                                     stage=stage.extend_name(
                                                               result.stage))
                        pick_poses = None  # Force searching for pick poses.
                    else:
                        node.pick_or_place_task.wait(goal.robot_name,
                                                     target_stage='approach')

                        # Search graspabilities for the next trial.
                        status, result = node.search_bin(goal.bin_id,
                                                         use_fine_parameters)
                        if status is GoalStatus.STATUS_ABORTED:
                            raise ActionServer.Error(
                                'Failed to search graspabilities',
                                stage=stage.name)
                        pick_poses = result.graspabilities.poses
                        # Wait until placing finished.
                        status, result = node.pick_or_place_task \
                                             .wait(goal.robot_name)
                        if status is GoalStatus.STATUS_ABORTED:
                            raise ActionServer.Error('Failed to place',
                                                     stage=stage.extend_name(
                                                               result.stage))
            elif status is GoalStatus.STATUS_UNKNOWN:  # no poses remained
                if not use_fine_parameters:
                    self.logger.warn('--- AttemptBin: failed at stage[%s], switch to fine parameters'
                                     % stage.name)
                    use_fine_parameters = True
                    pick_poses          = None  # Force searching.
                else:
                    self.logger.warn('--- AttemptBin: finished stage[%s] with status[%d], break'
                                     % (stage.name, status))
                    break
            else:  # GoalStatus.STATUS_ABORTED: returned from error handler
                use_fine_parameters = False
                pick_poses          = None  # Force searching for pick poses.

            if not goal.pick_all:
                break

        # [7] 'go_back_home' stage: Go back to home pose.
        with ActionServer.Stage(self, goal_handle, 'go_back_home',
                                stop) as stage:
            success = node.go_to_named_pose(goal.robot_name, 'home')
            if not success:
                raise ActionServer.Error('Failed to go back home',
                                         stage=stage.name)

        if goal.pick_all:
            # [8] 'place_tool' stage: Place tool.
            with ActionServer.Stage(self, goal_handle, 'place_tool',
                                    cancel_pick_or_place_tool) as stage:
                status, result = node.place_tool(goal.robot_name)
                if status is GoalStatus.STATUS_ABORTED:
                    raise ActionServer.Error('Failed to place tool!',
                                             stage=stage.extend_name(
                                                 result.stage))

            # [9] 'go_back_home_again' stage: Go back to home pose again.
            with ActionServer.Stage(self, goal_handle, 'go_back_home_again',
                                    stop) as stage:
                success = node.go_to_named_pose(goal.robot_name, 'home')
                if not success:
                    raise ActionServer.Error('Failed to go back home again!',
                                             stage=stage.name)

        goal_handle.succeed()
        return AttemptBin.Result(stage='')

    def _attempt_pick(self, goal, pick_poses: PoseArray,
                      fail_poses: List[PoseStamped], max_attempts: int):
        """ Repeat until a part successfully picked.

        Args:
          goal:         Goal request of this action.
          pick_poses:   Poses of candidate picking points.
          fail_poses:   Poses of picking points already failed.
          max_attempts: Maximum number of picking attempts.

        Returns:
          Tuple of GoalStatus, Result and index of the first unprocessed pose.
        """

        def _is_close_to_fail_poses(pose: PoseStamped, tolerance: float=0.005):
            def _are_close(p: Point, q: Point, tolerance: float):
                return abs(p.x - q.x) < tolerance and \
                       abs(p.y - q.y) < tolerance and \
                       abs(p.z - q.z) < tolerance

            for fail_pose in fail_poses:
                if _are_close(pose.pose.position, fail_pose.pose.position,
                              tolerance):
                    return True
            return False

        # Attempt to pick the item.
        part_id   = self.node.bin_props[goal.bin_id]['part_id']
        pose      = None
        nattempts = 0
        for p in pick_poses.poses:
            pose = PoseStamped(header=pick_poses.header, pose=p)
            if _is_close_to_fail_poses(pose):
                continue

            # Perform picking.
            status, result = self.node.pick(goal.robot_name, part_id, pose)

            # [A] Pick succeeded.
            if status is GoalStatus.STATUS_SUCCEEDED:
                return status, result, pose

            # [B] Pick failed.
            elif status is GoalStatus.STATUS_ABORTED:
                # [B-1] Error in moving to approach or pick pose.
                if result.stage in ('move', 'approach'):
                    fail_poses.append(pose)

                # [B-2] Error in departing from pick pose.
                elif result.stage == 'depart':
                    return status, result, pose

                # [B-3] Error in grasping.
                elif result.stage == 'verify':
                    fail_poses.append(pose)
                    nattempts += 1
                    if nattempts == max_attempts:
                        return status, result, pose

        # Here, no candidate picking points remained.
        return GoalStatus.STATUS_UNKNOWN, None, pose


#************************************************************************
#  class AttemptBinTask                                                 *
#************************************************************************
class AttemptBinTask(AttemptBinTaskClient):
    def __init__(self, node: Node, server_ns: str='attempt_bin'):
        super().__init__(node, server_ns)
        self._server = AttemptBinTaskServer(node, server_ns)
        self.wait_for_server()

    @property
    def server(self):
        return self._server
