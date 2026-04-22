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
import time
from rclpy.node            import Node
from aist_robotiq_msgs.msg import CModelStatus, CModelCommand

#########################################################################
#  class CModelBase                                                     #
#########################################################################
class CModelBase(Node):
    def __init__(self, name):
        super().__init__(name)
        slave_ids   = self.declare_parameter('slave_ids', [9]).value
        arg3fs      = self.declare_parameter('arg3f_grippers', [False]).value
        self._arg3f = dict(zip(slave_ids, arg3fs))
        self._pub   = self.create_publisher(CModelStatus, '~/status', 3)
        self._sub   = self.create_subscription(CModelCommand, '~/command',
                                               self.put_command, 3)
        self._timer = self.create_timer(0.05, self._timer_cb)
        self.get_logger().info('{slave_id: arg3f}: %s' % self._arg3f)

    def __del__(self):
        self.disconnect()           # (defined in derived class)

    def activate_devices(self):
        for slave_id in self._arg3f.keys():
            self.get_logger().info('activating device[slave_id=%d]' % slave_id)
            self.put_command(CModelCommand(r_sid=slave_id, r_act=0, r_gto=0))
            time.sleep(0.1)
            self.put_command(CModelCommand(r_sid=slave_id, r_act=1, r_gto=1))
            time.sleep(0.1)
            # for n in range(10):
            #     if self.get_status(slave_id).g_sta == 0x03:
            #         self.get_logger().info('activated device[slave_id=%d]'
            #                                % slave_id)
            #         break
            #     elif n == 9:
            #         self.get_logger().error('failed to activate device[slave_id=%d]'
            #                                % slave_id)
            #     time.sleep(0.5)
        self.get_logger().info('all devices activated')


    def _timer_cb(self):
        for slave_id in self._arg3f.keys():
            status = self.get_status(slave_id)  # (defined in derived class)
            if status is not None:
                self._pub.publish(status)  # Forward device status to controller

    def _clip_command(self, command):
        def clip(x, min_value, max_value):
            return min(max(min_value, x), max_value)

        command.r_sid = clip(command.r_sid, 1, 9)
        command.r_act = clip(command.r_act, 0, 1)
        command.r_mod = clip(command.r_mod, 0, 3)
        command.r_gto = clip(command.r_gto, 0, 1)
        command.r_atr = clip(command.r_atr, 0, 1)
        command.r_ard = clip(command.r_ard, 0, 1)
        command.r_icf = clip(command.r_icf, 0, 1)
        command.r_ics = clip(command.r_ics, 0, 1)
        command.r_pr  = clip(command.r_pr,  0, 255)
        command.r_sp  = clip(command.r_sp,  0, 255)
        command.r_fr  = clip(command.r_fr,  0, 255)
        command.r_prb = clip(command.r_prb, 0, 255)
        command.r_spb = clip(command.r_spb, 0, 255)
        command.r_frb = clip(command.r_frb, 0, 255)
        command.r_prc = clip(command.r_prc, 0, 255)
        command.r_spc = clip(command.r_spc, 0, 255)
        command.r_frc = clip(command.r_frc, 0, 255)
        command.r_prs = clip(command.r_prs, 0, 255)
        command.r_sps = clip(command.r_sps, 0, 255)
        command.r_frs = clip(command.r_frs, 0, 255)
        return command
