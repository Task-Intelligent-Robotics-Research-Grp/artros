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
import rospy, numpy as np
from actionlib          import SimpleActionServer, SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg  import PoseArray, Point, Quaternion
from aist_msgs.msg      import (SpiralSearchAction, SpiralSearchGoal,
                                SpiralSearchResult, SpiralSearchFeedback)
from tf                 import transformations as tfs

######################################################################
#  class SpiralSearch                                                #
######################################################################
class SpiralSearch(SimpleActionClient):
    def __init__(self, routines):
        SimpleActionClient.__init__(self, 'spiral_search', SpiralSearchAction)

        self._routines = routines
        self._server   = SimpleActionServer("spiral_search",
                                            SpiralSearchAction,
                                            self._execute_cb, False)
        self._server.register_preempt_callback(self._preempt_cb)
        self._server.start()
        self.wait_for_server()

    # Client stuffs
    def send_goal(self, robot_name, eef_link,
                  angle_increment, radius_increment, radius_max, timeout):
        SimpleActionClient.send_goal(self,
                                     SpiralSearchGoal(robot_name, eef_link,
                                                      angle_increment,
                                                      radius_increment,
                                                      radius_max,
                                                      speed, accel, tiemeout),
                                     done_cb, active_cb, self._feedback_cb)

    # Server stuffs
    def shutdown(self):
        self._server.__del__()

    def _execute_cb(self, goal):
        rospy.loginfo("*** Do spiral_searching ***")
        routines = self._routines
        path = routines.create_path(goal.robot_name,
                                    self._create_waypoints(
                                        eef_link,
                                        goal.angle_increment,
                                        goal.radius_increment,
                                        goal.radius_max),
                                    speed=goal.speed, accel=goal.accel,
                                    end_effector_link=eef_link)
        timeout_time = rospy.get_rostime() + goal.timeout
        while rospy.get_rostime() < timeout_time:


        self._server.set_succeeded(result, "Succeeded")

    def _preempt_cb(self):
        goal = self._server.current_goal.get_goal()
        self._routines.stop(goal.robot_name)
        self._server.set_preempted(SpiralSearchResult(SpiralSearchResult.PREEMPTED))
        rospy.logwarn('--- SpiralSearch cancelled. ---')

    def _create_waypoints(self, eef_link,
                          angle_increment, radius_increment, radius_max):
        poses = PoseArray()
        poses.header.frame_id = eef_link
        a = 0.0
        for r in np.arange(0.0, radius_max, radius_increment):
            poses.poses.append(Pose(Point(r*np.cos(a), r*np.sin(a), 0),
                                    Quaternion(0, 0, 0, 1)))
            a += np.radians(angle_increment)
        poses.poses.extend(list(reversed(copy.deepcopy(poses.poses[1:-1]))))

        return poses
