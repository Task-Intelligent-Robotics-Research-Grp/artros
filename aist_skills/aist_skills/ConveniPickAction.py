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
import rospy

from geometry_msgs.msg import PoseStamped
from actionlib         import SimpleActionServer, SimpleActionClient
from aist_msgs.msg     import (PickOrPlaceResult, PickOrPlaceFeedback,
                               ConveniPickAction, ConveniPickGoal)

######################################################################
#  class ConveniPick                                                 #
######################################################################
class ConveniPick(SimpleActionClient):
    def __init__(self, routines):
        SimpleActionClient.__init__(self, "conveni_pick", ConveniPickAction)

        self._routines              = routines
        self._current_robot_name    = None
        self._fail_poses            = []
        self._server                = SimpleActionServer("conveni_pick",
                                                         ConveniPickAction,
                                                         self._execute_cb,
                                                         False)
        self._server.register_preempt_callback(self._preempt_cb)
        self._server.start()
        self.wait_for_server()

    @property
    def current_robot_name(self):
        return self._current_robot_name

    # Client stuffs
    def send_goal(self, item_id, pick_all, max_attempts,
                  done_cb=None, active_cb=None):
        SimpleActionClient.send_goal(self,
                                     ConveniPickGoal(item_id, pick_all,
                                                     max_attempts),
                                     done_cb, active_cb)

    # Server stuffs
    def shutdown(self):
        self._server.__del__()

    def _execute_cb(self, goal):
        try_next = True
        self._clear_fail_poses()
        while try_next:
            try_next = self._conveni_pick(goal.item_id, goal.max_attempts)
            if not self._server.is_active():
                return
            if not goal.pick_all:
                break
        self._server.set_succeeded()
        rospy.loginfo('(ConveniPick) SUCCEEDED')

    def _conveni_pick(self, item_id, max_attempts):
        routines = self._routines
        try:
            item_props = routines._item_props[item_id]
            robot_name = item_props['robot_name']
        except KeyError as e:
            print(e)
            self._server.set_aborted()
            rospy.logerr('(ConveniPick) Unknown item_id[%s]', item_id)
            return False  # no items remained

        # Move to observation pose.
        routines.go_to_named_pose(robot_name, 'pick_ready')

        # Search for graspabilities.
        poses = routines.search_graspabilities(item_id).poses

        if not self._server.is_active():
            return False  # no items remained

        # Attempt to pick the item.
        nattempts = 0
        for p in poses.poses:
            if nattempts == max_attempts:
                break

            pose = PoseStamped(poses.header, p)
            if self._is_close_to_fail_poses(pose):
                continue

            # Perform picking.
            pick_result = routines.pick(robot_name, pose, item_id)
            if not self._server.is_active():
                return False

            # 1. Pick succeeded
            if pick_result == PickOrPlaceResult.SUCCESS:
                routines.go_to_named_pose(robot_name, 'pick_ready')
                routines.go_to_named_pose(robot_name, 'place_ready')

                # Begin placing and wait until reaching approach pose.
                routines.place_at_frame(robot_name, item_props['destination'],
                                        item_id, wait=False)
                routines.pick_or_place_wait_for_stage(
                    PickOrPlaceFeedback.APPROACHING)

                # Wait until placing finished.
                place_result = routines.pick_or_place_wait_for_result()
                return place_result == PickOrPlaceResult.SUCCESS

            # 2. Pick failed due to error in moving to approach/pick pose
            elif pick_result in (PickOrPlaceResult.MOVE_FAILURE,
                                 PickOrPlaceResult.APPROACH_FAILURE):
                self._fail_poses.append(pose)

            # 3. Pick failed due to error in departing from pick pose
            elif pick_result == PickOrPlaceResult.DEPARTURE_FAILURE:
                self._server.set_aborted()
                rospy.logerr('(ConveniPick) Failed to depart from pick/place pose')
                return False

            # 4. Pick failed due to error in grasping
            elif pick_result == PickOrPlaceResult.GRASP_FAILURE:
                self._fail_poses.append(pose)
                nattempts += 1

        return False

    def _preempt_cb(self):
        self._routines.pick_or_place_cancel_goal()
        self._server.set_preempted()
        rospy.logwarn('(ConveniPick) CANCELLED')

    # Utilities
    def _clear_fail_poses(self):
        self._fail_poses = []

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
