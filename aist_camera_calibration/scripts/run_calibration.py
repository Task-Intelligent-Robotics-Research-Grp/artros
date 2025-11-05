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
from rclpy.executors         import MultiThreadedExecutor
from aist_routines.base      import AISTBaseRoutines
from aist_camera_calibration\
    .CameraCalibrationAction import CameraCalibrationAction

######################################################################
#  class CameraCalibrationRoutines                                   #
######################################################################
class CameraCalibrationRoutines(AISTBaseRoutines):
    def __init__(self, name,
                 calibrator_ns='camera_calibrator',
                 server_ns='camera_calibration'):
        super().__init__(name)

        self._camera_calibration = CameraCalibrationAction(self,
                                                           calibrator_ns,
                                                           server_ns)

        cli_thread = threading.Thread(target=self.run)
        cli_thread.daemon = True
        cli_thread.start()

    @property
    def robot_name(self):
        return self._camera_calibration.robot_name

    @property
    def speed(self):
        return self._camera_calibration.speed

    def run(self):
        # Reset pose
        self.print_help_messages()
        print('')

        axis = 'Y'

        while rclpy.ok():
            prompt = '{:>5}:{}>> '.format(axis, self.format_pose(
                                                    self.get_current_pose(
                                                        self.robot_name)))
            key = input(prompt)
            _, axis, _ = self.interactive(key, self.robot_name, axis,
                                          self.speed)
        self.destroy_node()
        rclpy.shutdown()

    # interactive stuffs
    def print_help_messages(self):
        super().print_help_messages()
        print('=== Calibration commands ===')
        print('  init:   go to initial pose')
        print('  calib:  do calibration')
        print('  cancel: cancel calibration and then return to home pose')

    def interactive(self, key, robot_name, axis, speed):
        if key == 'init':
            self._camera_calibration.go_to_initpose()
        elif key == 'calib':
            self._camera_calibration.calibrate()
        elif key == 'cancel':
            self._camera_calibration.cancel()
            self._camera_calibration.wait()
            self.go_to_named_pose(robot_name, 'home')
        else:
            return super().interactive(key, robot_name, axis, speed)
        return robot_name, axis, speed

######################################################################
#  global functions                                                  #
######################################################################
def main():
    rclpy.init(args=sys.argv)

    node = CameraCalibrationRoutines('run_calibration')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

if __name__ == '__main__':
    main()
