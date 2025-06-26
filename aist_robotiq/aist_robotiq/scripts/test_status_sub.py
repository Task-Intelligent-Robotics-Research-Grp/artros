#!/usr/bin/env python3
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
import rclpy, sys, time, threading
import numpy as np
from rclpy.node            import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors       import (ExternalShutdownException,
                                   MultiThreadedExecutor)
from sensor_msgs.msg       import JointState
from aist_robotiq_msgs.msg import CModelStatus

#########################################################################
#  class TestStatusSub                                                  #
#########################################################################
class TestStatusSub(Node):
    def __init__(self, name):
        super().__init__(name)

        # Read configuration parameters
        self._min_position = self.declare_parameter('min_position', .810).value
        self._max_position = self.declare_parameter('max_position', .000).value
        self._min_velocity = self.declare_parameter('min_velocity', .013).value
        self._max_velocity = self.declare_parameter('max_velocity', .100).value
        self._min_effort   = self.declare_parameter('min_effort', 40.000).value
        self._max_effort   = self.declare_parameter('max_effort',100.000).value
        self._joint_name   = self.declare_parameter('joint_name',
                                                    'finger_joint').value

        self._status_sub      = self.create_subscription(
                                    CModelStatus, self.get_name() + '/status',
                                    self._status_cb, 10)
        self._joint_state_pub = self.create_publisher(
                                    JointState, '/joint_states', 1)

        # Position parameters to be calibrated
        self._min_gap_counts   = 255  # gap counts at full-close position
        self._max_gap_counts   = 0    # gap counts at full-open position
        self._calibration_step = 0    # ready for calibration

        self.get_logger().debug('started')

    def _status_cb(self, status):
        # Publish the joint_states for the gripper
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name         = [self._joint_name]
        joint_state.position     = [self._position(status)]
        self._joint_state_pub.publish(joint_state)

        self.get_logger().info('### status=%s' % status)

    def _position(self, status):
        return (status.g_po - self._min_gap_counts) * self.position_per_tick \
             + self._min_position

    @property
    def position_per_tick(self):
        return (self._max_position - self._min_position) \
             / (self._max_gap_counts - self._min_gap_counts)


if __name__ == '__main__':
    try:
        rclpy.init(args=sys.argv)
        test_status_sub = TestStatusSub('test_status_sub')
        # executor        = MultiThreadedExecutor(num_threads=4)
        # executor.add_node(test_status_sub)
        # executor.spin()
        rclpy.spin(test_status_sub)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass
