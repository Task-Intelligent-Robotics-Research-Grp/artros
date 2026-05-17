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
from rclpy.callback_groups        import MutuallyExclusiveCallbackGroup
from std_srvs.srv                 import Empty
from aist_msgs.srv                import (HandEyeCalibrationTakeSample,
                                          HandEyeCalibrationGetSampleList,
                                          HandEyeCalibrationComputeCalibration)
from task_wrappers.service_client import ServiceClient

#*********************************************************************
#  class HandEyeCalibratorClient                                     *
#*********************************************************************
class HandEyeCalibratorClient(object):
    def __init__(self, node, server_ns='handeye_calibrator'):
        super().__init__()

        self._cbg                 = MutuallyExclusiveCallbackGroup()
        self._take_sample         = ServiceClient(
                                        node,
                                        HandEyeCalibrationTakeSample,
                                        server_ns + '/take_sample',
                                        callback_group=self._cbg)
        self._get_sample_list     = ServiceClient(
                                        node,
                                        HandEyeCalibrationGetSampleList,
                                        server_ns + '/get_sample_list',
                                        callback_group=self._cbg)
        self._compute_calibration = ServiceClient(
                                        node,
                                        HandEyeCalibrationComputeCalibration,
                                        server_ns + '/compute_calibration',
                                        callback_group=self._cbg)
        self._reset               = ServiceClient(
                                        node,
                                        Empty, server_ns + '/reset',
                                        callback_group=self._cbg)

    def take_sample_async(self):
        self._take_sample.call(HandEyeCalibrationTakeSample.Request(),
                               timeout_sec=0.0)

    def wait_for_sample(self, timeout_sec=None):
        return self._take_sample.wait(timeout_sec)

    def get_sample_list(self):
        return self._get_sample_list.call(HandEyeCalibrationGetSampleList\
                                          .Request())

    def compute_calibration(self):
        return self._compute_calibration.call(
                   HandEyeCalibrationComputeCalibration.Request())

    def reset(self):
        return self._reset.call(Empty.Request())
