#!/usr/bin/env python
#
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
import rospy, threading
from geometry_msgs.msg   import WrenchStamped
from aist_utility.compat import *

#########################################################################
#  class TargetWrench                                                   #
#########################################################################
class TargetWrench(object):
    def __init__(self):
        super().__init__()

        robot_name = rospy.get_param('~robot_name', 'b_bot')
        self._wrench_pub = rospy.Publisher('/' + robot_name + '/target_wrench',
                                           WrenchStamped, queue_size=1)
        self._wrench = WrenchStamped()
        self._wrench.header.frame_id = robot_name + '_tool0'
        self._wrench.wrench.force.x  = 0.0
        self._wrench.wrench.force.y  = 0.0
        self._wrench.wrench.force.z  = 0.0
        self._wrench.wrench.torque.x = 0.0
        self._wrench.wrench.torque.y = 0.0
        self._wrench.wrench.torque.z = 0.0

        thread = threading.Thread(target=self._interactive)
        thread.start()

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self._wrench.header.stamp = rospy.Time.now()
            self._wrench_pub.publish(self._wrench)
            rate.sleep()

    def _interactive(self):
        while not rospy.is_shutdown():
            try:
                key = raw_input('> ')
                if key == 'q':
                    break
                else:
                    self._wrench.wrench.force.z  = float(key)
            except Exception as e:
                print(e.message)

if __name__ == '__main__':

    rospy.init_node('target_wrench', anonymous=True)

    target_wrench = TargetWrench()
    target_wrench.run()
