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
import rclpy
from ddynamic_reconfigure2 import ParameterClient

######################################################################
#  class CameraMultiplexerClient                                     #
######################################################################
class CameraMultiplexerClient(ParameterClient):
    def __init__(self, node, multiplexer_name='camera_multiplexer'):
        super().__init__(node, multiplexer_name)

    @property
    def camera_names(self):
        return self.get_parameters_sync(['camera_names'])[0]

    @property
    def active_camera_name(self):
        return self.get_parameters_sync(["active_camera_name"])[0]

    def activate_camera(self, camera_name):
        if camera_name in self.camera_names:
            return self.set_parameters_sync([('active_camera_name',
                                              camera_name)])[0].successful
        else:
            return False

######################################################################
#  class RealsenseMultiplexerClient                                  #
######################################################################
class RealsenseMultiplexerClient(CameraMultiplexerClient):

    class RealsenseCamera(ParameterClient):
        def __init__(self, node, camera_name):
            super().__init__(node, camera_name)

        @property
        def laser_power(self):
            return self.get_parameters_sync(
                       ['coded_light_depth_sensor.laser_power'])

        @laser_power.setter
        def laser_power(self, value):
            return self.set_parameters_sync(
                       [('coded_light_depth_sensor.laser_power',
                         value)])[0].successful

    def __init__(self, node, multiplexer_name='camera_multiplexer', value=16):
        super().__init__(node, multiplexer_name)
        self._node    = node
        self._cameras = None

    def activate_camera(self, camera_name, value=16):
        active_camera_name = self.active_camera_name

        if self._cameras is None:
            try:
                self._cameras \
                    = {camera_name:
                       RealsenseMultiplexerClient.RealsenseCamera(self._node,
                                                                  camera_name)
                       for camera_name in self.camera_names}
            except Exception as e:
                self._node.get_logger().error(str(e))
                self._node.get_logger().error("Cameras failed to initialize. Are the camera nodes started? Does /camera_multiplexer/camera_names contain unused cameras? camera_names: " + str(self.camera_names))
                return False
            for camera in self._cameras.values():
                camera.laser_power = 0
            self._cameras[active_camera_name].laser_power = value

        if not super().activate_camera(camera_name):
            return False
        self._cameras[active_camera_name].laser_power = 0
        self._cameras[camera_name].laser_power = value
        return True
