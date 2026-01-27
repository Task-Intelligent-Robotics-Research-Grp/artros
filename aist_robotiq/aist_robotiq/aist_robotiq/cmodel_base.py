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
from rclpy.node            import Node
from numpy                 import clip
from aist_robotiq_msgs.msg import CModelStatus, CModelCommand

#########################################################################
#  class CModelBase                                                     #
#########################################################################
class CModelBase(Node):
    def __init__(self, name):
        super().__init__(name)
        self._slave_ids = self.declare_parameter('slave_ids', [9]).value
        self._pub       = self.create_publisher(CModelStatus, '~/status', 3)
        self._sub       = self.create_subscription(CModelCommand, '~/command',
                                                   self.put_command, 3)
        self._timer     = self.create_timer(0.05, self._timer_cb)

    def __del__(self):
        self.disconnect()           # (defined in derived class)

    def _timer_cb(self):
        for slave_id in self._slave_ids:
            status = self.get_status(slave_id)  # (defined in derived class)
            self._pub.publish(status)   # Forward device status to controller

    def _clip_command(self, command):
        command.r_sid = clip(command.r_sid, 1, 9)
        command.r_act = clip(command.r_act, 0, 1)
        command.r_mod = clip(command.r_mod, 0, 3)
        command.r_gto = clip(command.r_gto, 0, 1)
        command.r_atr = clip(command.r_atr, 0, 1)
        command.r_ard = clip(command.r_ard, 0, 1)
        command.r_pr  = clip(command.r_pr,  0, 255)
        command.r_sp  = clip(command.r_sp,  0, 255)
        command.r_fr  = clip(command.r_fr,  0, 255)
        return command
