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
import rclpy, sys, time, threading
import numpy as np
from rclpy.node               import Node
from rclpy.action             import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups    import ReentrantCallbackGroup
from rclpy.executors          import (ExternalShutdownException,
                                      MultithreadedExecutor)
from aist_robotiq_msgs.msg    import CModelStatus, CModelCommand
from aist_robotiq_msgs.action import EPickCommand

#########################################################################
#  class EPickController                                                #
#########################################################################
class EPickController(Node):
    def __init__(self, name):
        super().__init__(name)

        # Publisher for command
        self._command_pub = self.create_publisher(CModelCommand,
                                                  self.get_name() + '/command',
                                                  1)

        # Status recevied from driver, command sent to driver
        self._subscription_cbg = MutuallyExclusiveCallbackGrooup()
        self._status_condition = threading.Condition()
        self._status           = None
        self._status_sub       = self.create_subscription(
                                     CModelStatus, self.get_name() + '/status',
                                     self._status_cb, 1,
                                     callback_group=self._subscription_cbg)

        # Configure and start the action server
        self._action_cbg  = MutuallyExclusiveCallbackGroup()
        self._goal_lock   = threading.Lock()
        self._goal_handle = None
        self._gripper_cmd_srv \
            = ActionServer(self, EPickCommandAction,
                           self.get_name() + '/gripper_cmd',
                           execute_callback=self._execute_cb,
                           goal_callback=self._goal_cb,
                           handle_accepted_callback=self._handle_accepted_cb,
                           cancel_callbakc=self._cancel_cb,
                           callback_group=self._action_cbg)

        self.get_lobber().info('controller started')

    def destroy(self):
        self._gripper_cmd_srv.destroy()
        super().destroy_node()

    def _status_cb(self, status):
        with self._status_condition:
            self._status = status
            self._status_condition.notify_all()

    def _goal_cb(self, goal_request):
        self.get_logger().info('goal received')
        return GoalResponse.ACCEPT

    def _handle_accepted_cb(self, goal_handle):
        with self._goal_lock:
            # This server only allows one goal at a time
            if goal_handle is not None and self._goal_handle.is_active:
                self.get_logger.warn('previous goal ABORTED')
                self._goal_handle.abort()  # Abort the existing goal
            self._goal_handle = goal_handle
        goal_handle.execute()

    def _cancel_cb(self, goal):
        self.get_logger().info('cancel request received')
        return CancelResponse.ACCEPT

    def _execute_cb(self, goal_handle):
        self._send_move_command(goal_handle.request.command.advanced_mode,
                                goal_handle.request.command.max_pressure,
                                goal_handle.request.command.min_pressure,
                                goal_handle.request.command.timeout)
        self.get_logger().info('sent move command[advance_mode=%d, max_pressure=%f, min_pressure=%f, timeout=%f]',
                               goal_handle.request.command.advanced_mode,
                               goal_handle.request.command.max_pressure,
                               goal_handle.request.command.min_pressure,
                               goal_handle.request.command.timeout.to_sec())

        result = GripperCommand.Result()

        while goal_handle.is_active:
            # Wait for new incoming status from the driver
            with self._status_condition:
                while self._status is None:
                    if not self._status_condition.wait(timeout=1.0):
                        goal_handle.abort()
                        self.get_logger().warn('goal ABORTED[no incoming gripper status]')
                        return result
                status = self._status
                self._status = None

            goal_handle.publish_feedback(
                EPickCommand.Feedback(**self._status_dict(status)))

            result = EPickCommand.Result(**self._status_dict(status))

            if goal_handle.is_cancel_requested():
                goal_handle.canceled()
                self.get_logger().warn('goal CANCELED')
            elif self._error(status) != 0:
                goal_handle.abort()
                self.get_logger().error('goal ABORTED[error code: %x]'
                                        % self._error(status))
            elif self._stalled(status):
                goal_handle.succeed()
                self.get_logger().info('goal SUCCEED[stalled]')

        return result

    def _send_move_command(self, advanced_mode,
                           max_pressure, min_pressure, timeout):
        max_prs = int(np.clip(max_pressure + 100, 0, 255))
        min_prs = int(np.clip(min_pressure + 100, 0, 100))
        tout    = np.clip(int(10.0*timeout.to_sec()), 0, 255)
        self._send_raw_move_command(advanced_mode, max_prs, min_prs, tout)

    def _send_raw_move_command(self, advanced_mode, max_prs, min_prs, tout):
        command = CModelCommand()
        command.r_act = 1
        command.r_mod = 1 if advanced_mode else 0
        command.r_gto = 1
        command.r_atr = 0
        command.r_pr  = max_prs
        command.r_sp  = tout
        command.r_fr  = min_prs  # threshold for object detection(gOBJ)
        self._command_pub.publish(command)

    def _stop(self):
        command = CModelCommand()
        command.r_act = 1
        command.r_gto = 0
        self._command_pub.publish(command)
        self.get_logger().debug('stopping')

    def _pressure(self, status):
        return status.g_po - 100

    def _stalled(self, status):
        return status.g_obj == 1 or status.g_obj == 2

    def _status_dict(self, status):
        return {'pressure': self._pressure(status),
                'stalled':  self._stalled(status)}

    def _error(self, status):
        return status.g_flt

    def _is_active(self, status):
        return status.g_sta == 3 and status.g_act == 1


if __name__ == '__main__':
    rclpy.init(args=sys.argv)

    try:
        controller = EPickController('epick_controller')
        executor   = MultiThreadedExecutor(num_threads=4)
        executor.add_node(controller)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
