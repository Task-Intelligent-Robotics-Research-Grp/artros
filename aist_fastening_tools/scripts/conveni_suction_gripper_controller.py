#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2023, National Institute of Advanced Industrial Science and Technology (AIST)
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
# Author: Toshio Ueshiba (t.ueshiba@aist.go.jp)
#
import threading, rospy
import serial
from actionlib                import SimpleActionServer
from aist_fastening_tools.msg import (SuctionToolCommandAction,
                                      SuctionToolCommandGoal,
                                      SuctionToolCommandResult,
                                      SuctionToolCommandFeedback)

#########################################################################
#  class ConveniSuctionGripperController                                #
#########################################################################
class ConveniSuctionGripperController(object):
    def __init__(self):
        super(ConveniSuctionGripperController, self).__init__()

        self._name = rospy.get_name()

        # Initialize ur_control table
        serial_port = rospy.get_param('~usb_port', '/dev/ttyUSB0')
        baud_rate   = rospy.get_param('~baud_rate', 9600)
        try:
            self._serial = serial.Serial(serial_port, baud_rate, timeout=1)
        except (OSError, serial.SerialException):
            rospy.logerr('(%s) failed to open serial port: %s',
                         self._name, serial_port)
            rospy.shutdown()

        # Create an action server for processing commands to suction tools.
        self._server = SimpleActionServer('~command', SuctionToolCommandAction,
                                          auto_start=False)
        self._server.register_goal_callback(self._goal_cb)
        self._server.register_preempt_callback(self._preempt_cb)
        self._server.start()
        rospy.loginfo('(%s) controller started', self._name)

    def _goal_cb(self):
        goal = self._server.accept_new_goal()

        rospy.loginfo('(%s) new goal ACCEPTED', self._name)

        # Check that preempt has not been requested by the client
        if self._server.is_preempt_requested():
            self._server.set_preempted()
            rospy.logwarn('(%s) pending goal CANCELED before proccessed',
                          self._name)
            return

        # Set states of suck and blow ports.
        self._send_command('1' if goal.suck else '0', goal.min_period)

        self._server.set_succeeded(SuctionToolCommandResult(goal.suck))
        rospy.loginfo('(%s) goal SUCCEEDED', self._name)

    def _preempt_cb(self):
        self._send_command('0')
        self._server.set_preempted(SuctionToolCommandResult(False))
        rospy.logwarn('(%s) active goal CANCELED by client', self._name)

    def _send_command(self, cmd, hold_time):
        start_time = rospy.get_rostime()
        while (rospy.get_rostime() - start_time) < hold_time:
            self._serial.write(cmd.encode())
            time.sleep(0.1)
            pass

    def _read(self):
        return self._serial.read(8)


#########################################################################
#  Entry point                                                          #
#########################################################################
if __name__ == '__main__':
    rospy.init_node('conveni_suction_gripper_controller')

    controller = ConveniSuctionGripperController()
    rospy.spin()
