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
import rclpy, sys, threading
from rclpy.node          import Node
from rclpy.duration      import Duration
from aist_robotiq.client import RobotiqGripper

#########################################################################
#  class TestGripperClient                                              #
#########################################################################
class TestGripperClient(Node):
    def __init__(self, name):
        super().__init__(name)

        gripper_name = self.declare_parameter('gripper_name',
                                              'a_bot_gripper').value
        self._gripper = RobotiqGripper(self, gripper_name + '_')
        self.get_logger().info('started')

        cli_thread = threading.Thread(target=self.interactive)
        cli_thread.daemon = True
        cli_thread.start()

    def interactive(self):
        def is_float(s):
            try:
                float(s)
            except ValueError:
                return False
            else:
                return True

        while rclpy.ok():
            print('==== Available commands ====')
            print('  g:         Grasp')
            print('  r:         Release')
            print('  <numeric>: Open gripper with a specified gap value')
            print('  c:         Cancel motion')
            print('  w:         Wait until goal completed')
            print('  m:         Switch mode')
            print('  q:         Quit\n')

            key = input('>> ')
            if key == 'g':
                self._gripper.grasp(timeout=None)
            elif key == 'r':
                self._gripper.release(timeout=None)
            elif is_float(key):
                self._gripper.move(float(key), timeout=None)
            elif key == 'c':
                self._gripper.cancel()
            elif key == 'w':
                status, result = self._gripper.wait(timeout=Duration(seconds=10))
                print(result)
            elif key == 'v':
                velocity = float(input('  velocity: '))
                success = self._gripper.set_velocity(velocity)
                print('%s to set velocity'
                      % ('succeeded' if success else 'failed'))
            elif key == 'm':
                mode = int(input('  mode(0: BASIC, 1: PINCH, 2: WIDE, 3: SCISSOR, 4: ICF, 5: ICS): '))
                success = self._gripper.set_mode(mode)
                print('%s to set mode' % ('succeeded' if success else 'failed'))
            elif key=='q':
                break
            else:
                print('unknown command: %s' % key)

        self.destroy_node()
        rclpy.shutdown()

def main():
    rclpy.init(args=sys.argv)

    test = TestGripperClient('test_gripper_client')
    rclpy.spin(test)

if __name__ == '__main__':
    main()
