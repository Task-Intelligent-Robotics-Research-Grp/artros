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
import rclpy, os, threading
import numpy as np
from rclpy.node            import Node
from rclpy.action          import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors       import (ExternalShutdownException,
                                   MultiThreadedExecutor)
from sensor_msgs.msg       import JointState
from control_msgs.action   import GripperCommand
from aist_robotiq_msgs.msg import CModelStatus, CModelCommand
from aist_robotiq_msgs.srv import SetVelocity

#########################################################################
#  class CModelController                                               #
#########################################################################
class CModelController(Node):
    def __init__(self):
        super().__init__('cmodel_controller')

        # Read configuration parameters
        self._min_position = self.declare_parameter('min_position', .810).value
        self._max_position = self.declare_parameter('max_position', .000).value
        self._min_velocity = self.declare_parameter('min_velocity', .013).value
        self._max_velocity = self.declare_parameter('max_velocity', .100).value
        self._min_effort   = self.declare_parameter('min_effort', 40.000).value
        self._max_effort   = self.declare_parameter('max_effort',100.000).value
        self._joint_name   = self.declare_parameter('joint_name',
                                                    'finger_joint').value

        # Velocity set by service server
        self._velocity         = 0.5*(self._min_velocity + self._max_velocity)
        self._set_velocity_srv = self.create_service(SetVelocity,
                                                     self.get_name() \
                                                     + '/set_velocity',
                                                     self._set_velocity_cb)

        # Status recevied from the driver and command sent to the driver
        self._status_condition = threading.Condition()
        self._status           = None
        self._status_sub       = self.create_subscription(
                                     CModelStatus, self.get_name() + '/status',
                                     self._status_cb, 1,
                                     callback_group=ReentrantCallbackGroup())
        self._command_pub      = self.create_publisher(
                                     CModelCommand,
                                     self.get_name() + '/command', 1)
        self._joint_state_pub  = self.create_publisher(
                                     JointState, '/joint_states', 1)
        self._goal_rPR         = 0

        # Position parameters to be calibrated
        self._min_gap_counts   = 255  # gap counts at full-close position
        self._max_gap_counts   = 0    # gap counts at full-open position
        self._calibration_step = 0    # ready for calibration

        # Configure and start the action server
        self._goal_lock   = threading.Lock()
        self._goal_handle = None
        self._gripper_cmd_srv \
            = ActionServer(self, GripperCommand,
                           self.get_name() + '/gripper_cmd',
                           execute_callback=self._execute_cb,
                           goal_callback=self._goal_cb,
                           handle_accepted_callback=self._handle_accepted_cb,
                           cancel_callback=self._cancel_cb,
                           callback_group=ReentrantCallbackGroup())

        # Calibrate gripper
        self._sleep(2.0)              # wait for server comes up
        self._calibrate()

        self.get_logger().debug('started')

    def destroy(self):
        self._gripper_cmd_srv.destroy()
        super().destroy_node()

    def _set_velocity_cb(self, req, res):
        self._velocity = req.velocity
        res.success = True
        return res

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
        self._goal_rPR \
            = self._send_move_command(goal_handle.request.command.position,
                                      self._velocity,
                                      goal_handle.request.command.max_effort)
        self.get_logger().info('sent move command[position=%f, velocity=%f, max_effort=%f]',
                               goal_handle.request.command.position,
                               self._velocity,
                               goal.handle.request.command.max_effort)

        while goal_handle.is_active:
            # Wait for new incoming status from the driver
            with self._status_condition:
                while self._status is None:
                    self._status_condition.wait()
                status = self._status
                self._status = None

            goal_handle.publish_feedback(
                GripperCommand.Feedback(*self._status_values(status)))

            result = GripperCommand.Result(*self._status_values(status))

            if goal_handle.is_cancel_requested():
                goal_handle.canceled()
                self.get_logger().warn('goal CANCELED')
            elif self._error(status) != 0:
                goal_handle.abort()
                self.get_logger().error('goal ABORTED[error code: %x]',
                                        self._error(status))
            elif self._reached_goal(status):
                goal_handle.succeed()
                self.get_logger().info('goal SUCCEED[reached goal]')
            elif self._stalled(status):
                goal_handle.succeed()
                self.get_logger().info('goal SUCCEED[stalled]')

        return result

    def _status_cb(self, status):
        # Publish the joint_states for the gripper
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now()
        joint_state.name         = [self._joint_name]
        joint_state.position     = [self._position(status)]
        self._joint_state_pub.publish(joint_state)

        # Handle calibration process if not moving
        if self._is_active(status) and not self._is_moving(status):
            if self._calibration_step == 1:
                self.get_logger().info("calibration step 1: start calibration")
                self._calibration_step = 2
                self._send_raw_move_command(0, 64, 1)    # full-open
                self._sleep(0.5)
            elif self._calibration_step == 2:
                self._max_gap_counts = status.gPO        # record at full-open
                self.get_logger().info("calibration step 2: gap[%d]@full-open",
                                       self._max_gap_counts)
                self._calibration_step = 3
                self._send_raw_move_command(255, 64, 1)  # full-close
                self._sleep(0.5)
            elif self._calibration_step == 3:
                self._min_gap_counts = status.gPO        # record at full-close
                self.get_logger().info("calibration step 3: gap[%d]@full-close",
                                       self._min_gap_counts)
                self._calibration_step = 0
                self._send_raw_move_command(0, 64, 1)    # full-open
                self.get_logger().info('calibrated to [%d, %d]',
                                       self._min_gap_counts,
                                       self._max_gap_counts)

        with self._status_condition:
            self._status = status
            self._status_condition.notifyAll()

    def _calibrate(self):
        self._calibration_step = 1

    def _send_move_command(self, position, velocity, effort):
        # print('*** _send_move_command: position=%f, velocity=%f, effort=%f'
        #       % (position, velocity, effort))
        pos = np.clip(int((position - self._min_position)
                          / self.position_per_tick + self._min_gap_counts),
                      self._max_gap_counts, self._min_gap_counts)
        vel = np.clip(int((velocity - self._min_velocity)
                          / self.velocity_per_tick),
                      0, 255)
        eff = np.clip(int((effort - self._min_effort) / self.effort_per_tick),
                      0, 255)
        self._send_raw_move_command(pos, vel, eff)
        return pos

    def _send_raw_move_command(self, pos, vel, eff):
        # print('*** _send_raw_move_command: pos=%d, vel=%d, eff=%d'
        #       % (pos, vel, eff))
        command = CModelCommand()
        command.r_act = 1
        command.r_gto = 1
        command.r_pr  = pos
        command.r_sp  = vel
        command.r_fr  = eff
        self._command_pub.publish(command)

    def _stop(self):
        command = CModelCommand()
        command.r_act = 1
        command.r_gto = 0
        self._command_pub.publish(command)
        self.get_logger().debug('stopping')

    def _position(self, status):
        return (status.gPO - self._min_gap_counts) * self.position_per_tick \
             + self._min_position

    def _effort(self, status):
        return status.gCOU * self.effort_per_tick + self._min_effort

    def _stalled(self, status):
        # After the goal accepted in _goal_cb(), status.g_pr does not
        # correctly reflects the requested position if _status_cb() is
        # called before _send_move_command(). Thus we have to use
        # self._goal_rPR instead of status.gPR.
        return (status.gOBJ == 1 and status.gPO > self._goal_rPR + 1) or \
               (status.gOBJ == 2 and status.gPO + 1 < self._goal_rPR)

    def _reached_goal(self, status):
        # ibid
        return status.gOBJ == 3 and abs(status.gPO - self._goal_rPR) <= 1

    def _status_values(self, status):
        return self._position(status), self._effort(status), \
               self._stalled(status),  self._reached_goal(status)

    def _error(self, status):
        return status.g_flt

    def _is_active(self, status):
        return status.g_sta == 3 and status.g_act == 1

    def _is_moving(self, status):
        return status.g_gto == 1 and status.g_obj == 0

    def _sleep(self, sec):
        rate = self.create_rate(1.0/sec)
        rate.sleep()
        self.destroy_rate(rate)

    @property
    def position_per_tick(self):
        return (self._max_position - self._min_position) \
             / (self._max_gap_counts - self._min_gap_counts)

    @property
    def velocity_per_tick(self):
        return (self._max_velocity - self._min_velocity) / 255

    @property
    def effort_per_tick(self):
        return (self._max_effort - self._min_effort) / 255

if __name__ == '__main__':
    try:
        rclpy.init()
        controller      = CModelController()
        executor        = MultiThreadedExecutor(num_threads=4)
        executor.add_node(controller)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
