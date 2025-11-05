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
from rclpy.node                     import Node
from rclpy.executors                import MultiThreadedExecutor
from tf2_ros.buffer                 import Buffer
from tf2_ros.transform_listener     import TransformListener
from aist_camera_calibration.client import CameraCalibrationClient

######################################################################
#  class PlaneCalibrationRoutines                                    #
######################################################################
class PlaneCalibrationRoutines(Node):
    def __init__(self, name, calibrator_ns='camera_calibrator',):
        super().__init__(name)

        # Create TransformListener
        self._tf2_buffer   = Buffer()
        self._tf2_listener = TransformListener(self._tf2_buffer, self)
        self._calib_dir    = self.declare_parameter('calibration_dir',
                                                    '').value
        self._calibrator   = CameraCalibratorClient(node, calibrator_ns)

        cli_thread = threading.Thread(target=self.run)
        cli_thread.daemon = True
        cli_thread.start()

    def run(self):
        # Reset pose
        while rclpy.ok():
            print('\n  q  : quit program')
            print('  RET: take sample')
            print('  g  : get sample list')
            print('  c  : compute calibration')
            print('  r  : reset and discard all samples')

            prompt = '>> '
            key = input(prompt)
            if key == 'q':
                break
            elif key == 'g':
                self.get_sample_list()
            elif key == 'c':
                self.compute_calibration()
            elif key == 'r':
                self._calibrator.reset()
            else:
                self.take_sample()

        self.destroy_node()
        rclpy.shutdown()

    def take_sample(self):
        result = self._calibrator.wait_for_samle(
                     self._calibrator.take_sample_async())
        self.get_logger().info(result.message)
        for correspondences in result.correspondences_set:
            print('  [%s] %d point correspondences w.r.t. %s'
                  % (correspondences.camera_name,
                     len(correspondences.correspondences),
                     correspondences.reference_frame))

    def get_sample_list(self):
        res = self._calibrator.get_sample_list()
        self.get_logger().info(res.message)
        for correspondences_set in res.correspondences_sets:
            for correspondences in correspondences_set.correspondences_set:
                print('  [%s] %d point correspondences w.r.t. %s'
                      % (correspondences.camera_name,
                         len(correspondences.correspondences),
                         correspondences.reference_frame))
            print('')

    def compute_calibration(self):
        self._save_calibration(self, self._calibrator.compute_calibration())

    def _save_calibration(self, res):
        if not res.success:
            self._logger.error('calibration failed: %s' % res.message)
            return

        for camera_name, camera_info, camera_pose in zip(res.camera_names,
                                                         res.intrinsics,
                                                         res.camera_poses):
            print('=== estimated pose of % ===' % camera_name)
            print('[{:.4f}, {:.4f}, {:.4f}; {:.2f}, {:.2f}. {:.2f}]'\
                  .format(*self._node.xyzrpy_from_pose(camera_pose)))

            # Convert camera pose to xyz-rpy representation.
            data = {'parent': camera_pose.header.frame_id,
                    'child' : camera_info.header.frame_id,
                    'origin': self._node.xyzrpy_from_pose(camera_pose)}

            # Save camera pose.
            dirname  = filepath_from_url(self._calib_dir)
            filename = dirname + '/' + camera_name + '.yaml'
            with open(filename, mode='w') as file:
                yaml.dump(data, file, default_flow_style=False)
            self._logger.info('saved camera extrinsiscs in [%s]' % filename)

            # Save camera_info.
            filename = dirname + '/' + camera_name + '-camera_info.yaml'
            saveCalibration(camera_info, filename, camera_name)
            self._logger.info('saved camera intrinsiscs in [%s]' % filename)


######################################################################
#  global functions                                                  #
######################################################################
def main():
    rclpy.init(args=sys.argv)

    node = PlaneCalibrationRoutines('run_plane_calibration')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

if __name__ == '__main__':
    main()
