#!/usr/bin/env python3
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
import sys, re
import rclpy
import xml.etree.ElementTree as ET

from rclpy.node   import Node
from rclpy.qos    import QoSProfile, DurabilityPolicy
from std_msgs.msg import String

#########################################################################
#  class Ros2ControlAppender                                            #
#########################################################################
class Ros2ControlAppender(Node):
    def __init__(self, name):
        super().__init__(name)

        # Load XML descriptions of ros2_control from parameter and parse them.
        rds = self.declare_parameter('ros2_control_descriptions', '').value
        indices = [rd.start() for rd in re.finditer('<\?xml', rds)]
        indices.append(len(rds))
        self._eds = [ET.fromstring(rds[indices[i]:indices[i+1]])
                     for i in range(len(indices) - 1)]

        # Create subscriber and publisher.
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._sub = self.create_subscription(String, 'robot_description_in',
                                             self._robot_description_cb, qos)
        self._pub = self.create_publisher(String, 'robot_description', qos)

        self.get_logger().info('initialized')

    def _robot_description_cb(self, rd_msg):
        self.get_logger().info('received robot_description')
        rd = ET.fromstring(rd_msg.data)
        for ed in self._eds:
            for i in range(len(ed)):
                rd.append(ed[i])
        self._pub.publish(String(data=ET.tostring(rd, encoding='unicode')))


#########################################################################
#  entry point                                                          #
#########################################################################
def main():
    try:
        rclpy.init(args=sys.argv)

        node = Ros2ControlAppender('append_ros2_control')
        rclpy.spin(node)
    except Exception as e:
        print('*** Terminate the node due to exception: %s' % e)

if __name__ == '__main__':
    main()
