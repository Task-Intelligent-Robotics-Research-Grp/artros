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
import sys
import rclpy
import xml.etree.ElementTree as ET

from rclpy.node   import Node
from std_msgs.msg import String

#########################################################################
#  class XmlAdder                                                       #
#########################################################################
class XmlAdder(Node):
    def __init__(self, name):
        super().__init__(name)

        self._ed  = ET.fromstring(self.declare_parameter('extra_description',
                                                         '').value)
        self._sub = self.create_subscription(String, 'robot_description_in',
                                             self._robot_description_cb, 4)
        self._pub = self.create_publisher(String, 'robot_description', 4)

        self.get_logger().info('initialized')

    def _robot_description_cb(self, rd_msg):
        rd = ET.fromstring(rd_msg.data)
        rd.append(self._ed[0])
        self._pub.publish(String(data=ET.tostring(rd)))


#########################################################################
#  entry point                                                          #
#########################################################################
def main():
    try:
        rclpy.init(args=sys.argv)

        node = XmlAdder('xml_adder')
        rclpy.spin(node)
    except Exception as e:
        print('*** Terminate the node due to exception: %s' % e)

if __name__ == '__main__':
    main()
