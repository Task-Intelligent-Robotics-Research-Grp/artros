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
import rclpy, time
from rclpy.node            import Node
from rclpy.parameter       import Parameter, parameter_value_to_python
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rcl_interfaces.srv    import GetParameters, SetParameters
from rcl_interfaces.msg    import (ParameterDescriptor, ParameterType,
                                   IntegerRange, FloatingPointRange)

######################################################################
#  class CameraMultiplexerClient                                     #
######################################################################
class CameraMultiplexerClient(object):
    def __init__(self, node, server='camera_multiplexer'):
        super().__init__()
        self._logger     = node.get_logger()

        self._get_cbg    = MutuallyExclusiveCallbackGroup()
        self._get_client = node.create_client(GetParameters,
                                              server + '/get_parameters',
                                              callback_group=self._get_cbg)
        while not self._get_client.wait_for_service(timeout_sec=2.0):
            self._logger.info('service[%s/get_parameters] not available, waiting...' % server)
        self._get_future = None

        self._set_cbg    = MutuallyExclusiveCallbackGroup()
        self._set_client = node.create_client(SetParameters,
                                              server + '/set_parameters',
                                              callback_group=self._set_cbg)
        while not self._get_client.wait_for_service(timeout_sec=2.0):
            self._logger.info('service[%s/set_parameters] not available, waiting...' % server)
        self._set_future = None

        self._logger.info('initialized CameraMultiplexerClient')

    @property
    def camera_names(self):
        return self._get_parameters(['camera_names']) \
                   .values[0].string_array_value

    @property
    def active_camera(self):
        return self._get_parameters(["active_camera"]) \
                   .values[0].string_value

    def activate_camera(self, camera_name):
        if camera_name in self.camera_names:
            self._set_parameters(['active_camera'], [camera_name])
            time.sleep(0.2)
            return True
        else:
            return False

    def _get_parameters(self, param_names):
        self._get_future = None
        req = GetParameters.Request(names=param_names)
        self._get_client.call_async(req) \
                        .add_done_callback(self._get_parameters_cb)
        while self._get_future is None or not self._get_future.done():
            self._logger.info('_get_parameters(): waiting...')
            time.sleep(0.1)
        return self._get_future.result()

    def _get_parameters_cb(self, future):
        self._get_future = future

    def _set_parameters(self, param_names, param_values):
        self._set_future = None
        req = SetParameters.Request()
        req.parameters = [Parameter(param_name, param_value)
                          for param_name, param_value in zip(param_names,
                                                             param_values)]
        self._set_client.call_async(req) \
                        .add_done_callback(self._set_parameters_cb)
        while self._set_future is None or not self._set_future.done():
            self._logger.info('_set_parameters(): waiting...')
            time.sleep(0.1)
        return self._set_future.result()

    def _set_parameters_cb(self, future):
        self._set_future = future

######################################################################
#  class RealSenseMultiplexerClient                                  #
######################################################################
class RealSenseMultiplexerClient(CameraMultiplexerClient):

    class RealSenseCamera(object):
        def __init__(self, camera_name):
            super().__init__()
            self._ddr_client = dynamic_reconfigure.client.Client(
                                   camera_name + '/coded_light_depth_sensor',
                                   timeout=5.0)

        @property
        def laser_power(self):
            return self._ddr_client.get_configuration()['laser_power']

        @laser_power.setter
        def laser_power(self, value):
            self._ddr_client.update_configuration({'laser_power': value})

    def __init__(self, node, server='camera_multiplexer', value=16):
        try:
            super().__init__(node, server)
            self._cameras = dict(zip(self.camera_names,
                                 [RealSenseMultiplexerClient.RealSenseCamera(
                                     camera_name)
                                  for camera_name in self.camera_names]))
        except Exception as e:
            self._logger.error(str(e))
            self._logger.error("Cameras failed to initialize. Are the camera nodes started? Does /camera_multiplexer/camera_names contain unused cameras? camera_names: " + str(self.camera_names))
        for camera in self._cameras.values():
            camera.laser_power = 0
        self._cameras[self.active_camera].laser_power = value

    def activate_camera(self, camera_name, value=16):
        self._cameras[self.active_camera].laser_power = 0
        super().activate_camera(camera_name)
        self._cameras[self.active_camera].laser_power = value
