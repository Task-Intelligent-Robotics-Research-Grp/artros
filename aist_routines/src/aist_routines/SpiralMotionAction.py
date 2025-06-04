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
import rospy, copy, numpy as np
from actionlib          import SimpleActionServer, SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg  import PoseArray, Pose, Point, Quaternion
from aist_msgs.msg      import (SpiralMotionAction, SpiralMotionGoal,
                                SpiralMotionResult, SpiralMotionFeedback)
from tf                 import transformations as tfs

######################################################################
#  class SpiralMotion                                                #
######################################################################
class SpiralMotion(SimpleActionClient):
    def __init__(self, routines):
        SimpleActionClient.__init__(self, 'spiral_motion', SpiralMotionAction)

        self._routines = routines
        self._success  = False
        self._server   = SimpleActionServer('spiral_motion',
                                            SpiralMotionAction,
                                            self._execute_cb, False)
        self._server.register_preempt_callback(self._preempt_cb)
        self._server.start()
        self.wait_for_server()

    # Client stuffs
    def send_goal(self, robot_name, eef_link='',
                  npoints=36, angle_increment=30.0,
                  radius_x_max=0.005, radius_y_max=0.0005,
                  roll_npoints=3, roll_max=15.0,
                  speed=0.01, accel=1.0, timeout=rospy.Duration(30.0),
                  done_cb=None, active_cb=None):
        SimpleActionClient.send_goal(self,
                                     SpiralMotionGoal(robot_name, eef_link,
                                                      npoints,
                                                      angle_increment,
                                                      radius_x_max,
                                                      radius_y_max,
                                                      roll_npoints,
                                                      roll_max,
                                                      speed, accel, timeout),
                                     done_cb, active_cb)

    # Server stuffs
    def shutdown(self):
        self._server.__del__()

    def _execute_cb(self, goal):
        rospy.loginfo('(SpiralMotion) goal ACCEPTED')
        waypoints = self._create_waypoints(goal.eef_link,
                                           goal.npoints,
                                           goal.angle_increment,
                                           goal.radius_x_max,
                                           goal.radius_y_max,
                                           goal.roll_npoints,
                                           goal.roll_max)
        timeout_time = rospy.get_rostime() + goal.timeout
        while goal.timeout <= rospy.Duration() or \
              rospy.get_rostime() < timeout_time:
            if not self._server.is_active():
                return
            self._routines.go_along_poses(goal.robot_name, waypoints,
                                          speed=goal.speed, accel=goal.accel,
                                          end_effector_link=goal.eef_link)
        self._server.set_aborted()
        rospy.logerr('(SpiralMotion) goal ABORTED: timeout[%f]',
                     goal.timeout.to_sec())

    def _preempt_cb(self):
        goal = self._server.current_goal.get_goal()
        self._routines.stop(goal.robot_name)
        self._server.set_preempted()
        rospy.logwarn('(SpiralMotion) goal CANCELLED')

    def _create_waypoints(self, eef_link, npoints, angle_increment,
                          radius_x_max, radius_y_max, roll_npoints, roll_max):
        poses = PoseArray()
        poses.header.frame_id = eef_link

        a              = 0.0
        roll           = 0.0
        roll_increment = roll_max/roll_npoints
        for n in range(0, npoints + 1):
            rx = n*radius_x_max/npoints
            ry = n*radius_y_max/npoints
            poses.poses.append(Pose(Point(rx*np.cos(a), ry*np.sin(a), 0),
                                    Quaternion(0, 0, np.sin(roll/2),
                                               np.cos(roll/2))))
            a += np.radians(angle_increment)
            if (n + roll_npoints) % (2*roll_npoints) == 0:
                roll_increment = -roll_increment
            roll += np.radians(roll_increment)
        poses.poses.extend(list(reversed(copy.deepcopy(poses.poses[1:-1]))))

        return poses
