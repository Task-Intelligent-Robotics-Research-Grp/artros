#! /usr/bin/env python
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
import numpy as np
from rclpy.node                   import Node
from rclpy.duration               import Duration
from rclpy.executors              import (ExternalShutdownException,
                                          SingleThreadedExecutor,
                                          MultiThreadedExecutor)
from rclpy.callback_groups        import (MutuallyExclusiveCallbackGroup,
                                          ReentrantCallbackGroup)
from rclpy.action                 import (ActionServer,
                                          GoalResponse, CancelResponse)
from sensor_msgs.msg              import JointState
from control_msgs.action          import GripperCommand
from dynamixel_workbench_msgs.msg import DynamixelState, DynamixelStateList
from dynamixel_workbench_msgs.srv import DynamixelCommand

#########################################################################
#  class PrecisionGripperController                                     #
#########################################################################
class PrecisionGripperController(Node):
    def __init__(self, name, *kargs, **kwargs):
        super().__init__(name, *kargs, **kwargs)

        # Read motor id
        self._id = self.declare_parameter('~ID', 1).value

        # Read timeout value for checking stalled state
        self._stall_timeout = Duration(seconds=self.declare_parameter(
                                                   'stall_timeout', 1).value)

        # Read configuration parameters
        self._min_position = self.declare_parameter('min_position',
                                                    0.000).value
        self._max_position = self.declare_parameter('max_position',
                                                    0.010).value
        self._max_effort   = self.declare_parameter('max_effort',
                                                    0.5).value

        # Read servo parameters
        self._min_pos = self.declare_parameter('min_position_count',
                                               2300).value
        self._max_pos = self.declare_parameter('max_position_count',
                                               2050).value
        self._min_cur = self.declare_parameter('min_effort_count',  3).value
        self._max_cur = self.declare_parameter('max_effort_count', 13).value

        # Create a subscriber for receiving state of Dynamixel driver.
        driver_ns = self.declare_parameter('driver_ns',
                                           'precision_gripper_driver').value
        self._dxl_state_cbg       = MutuallyExclusiveCallbackGroup()
        self._dxl_state_condition = threading.Condition()
        self._dxl_state           = None
        self._dxl_state_sub       = self.create_subscription(
                                        DynamixelStateList,
                                        driver_ns + '/dynamixel_state',
                                        self._state_list_cb, 10,
                                        callback_group=self._dxl_state_cbg)

        # Create a service client for sending command to _Dxl driver
        self._response_condition = threading.Condition()
        self._response           = None
        self._dxl_command        = self.create_client(DynamixelCommand,
                                                      driver_ns
                                                      + '/dynamixel_command')
        if not self._dxl_command.wait_for_service(1.0):
            raise RuntimeError

        # Publish joint state
        self._joint_state_pub = self.create_publisher(JointState,
                                                      '/joint_states', 1) \
                                if self.declare_parameter(
                                        'publish_joint_states', True).value \
                                else None

        # Define the action
        self._action_cbg = MutuallyExclusiveCallbackGroup()
        self._server \
            = ActionServer(self, GripperCommand, '~/gripper_cmd',
                           execute_callback=self._execute_cb,
                           goal_callback=self._goal_cb,
                           handle_accepted_callback=self._handle_accepted_cb,
                           cancel_callback=self._cancel_cb,
                           callback_group=self._action_cbg)
        self._goal_pos = 0

        self.get_logger().info('controller started')

    def _state_list_cb(self, state_list):
        # Keep new state
        states = [state for state in state_list.dynamixel_state
                  if state.id == self._id]
        if not states:
            self.get_logger().error('dynamixel state with ID=%i not found in state list' % self._id)
            return

        with self._dxl_state_condition:
            self._dxl_state = states[0]
            self._dxl_state_condition.notify_all()

        # Publish joint state
        if self._joint_state_pub is not None:
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name     = [self._dxl_state.name  + '_finger_joint']
            joint_state.position = [self._position()]
            joint_state.velocity = [0.0]
            joint_state.effort   = [self._effort()]
            self._joint_state_pub.publish(joint_state)

    def _goal_cb(self, goal_request):
        self.get_logger().info('goal received[position=%f, max_effort=%f]'
                               % (goal_request.command.position,
                                  goal_request.command.max_effort))
        return GoalResponse.ACCEPT

    def _handle_accepted_cb(self, goal_handle):
        # with self._goal_lock:
        #     if self._goal_handle is not None and self._goal_handle.is_active:
        #         self.get_logger().warn('previous goal CANCELED')
        #         self._goalhandle.canceled()
        #     self._goal_handle = goal_hande
        goal_handle.execute()

    def _cancel_cb(self, goal):
        self.get_logger().info('cancel request received')
        return CancelResponse.ACCEPT

    def _execute_cb(self, goal_handle):
        self._send_move_command(goal_handle.request.command.position,
                                goal_handle.request.command.max_effort)

        result = GripperCommand.Result()

        while goal_handle.is_active:
            # Wait for new incoming status from the driver
            with self._dxl_state_condition:
                while self._dxl_state is None:
                    if not self._dxl_state_condition.wait(timeout=1.0):
                        goal_handle.abort()
                        self.get_logger().error('goal ABORTED[no incoming dynamixel state]')
                        return result
                dxl_state = self._dxl_state
                self._dxl_state = None

            goal_handle.publish_feedback(
                GripperCommand.Feedback(**self._dxl_state_dict(dxl_state)))

            result = GripperCommand.Result(**self._dxl_state_dict(dxl_state))

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().warn('goal CANCELED')
            elif self._is_moving():
                self._last_movement_time = self.get_clock().now()
            elif self._reached_goal(dxl_state):
                goal_handle.succeed()
                self.get_logger().info('goal SUCCEED[reached goal]')
            elif self._stalled(dxl_state):
                goal_handle.succeed()
                self.get_logger().info('goal SUCCEED[stalled]')

        return result

    def _send_move_command(self, position, effort):
        pos = np.clip(int((position - self._min_position) /
                          self.position_per_tick + self._min_pos),
                      self._max_pos, self._min_pos)
        cur = np.clip(int(effort / self.effort_per_tick),
                      -self._max_cur, self._max_cur)
        if abs(cur) < self._min_cur:
            pos_now = np.int32(self._position())
            cur = self._min_cur if pos > pos_now else -self._min_cur
        self.get_logger().info('** Cmd(pos=%i, cur=%i) for position=%f, effort=%f' % (pos, cur, position, effort))
        self._set_value('Goal_Current',  cur)
        self._set_value('Goal_Position', pos)
        return pos

    def _set_value(self, addr_name, value):
        self._response = None
        self._dxl_command.call_async(
            DynamixelCommand.Request(command='', id=self._id,
                                     addr_name=addr_name, value=value)) \
                         .add_done_callback(self._get_response_cb)
        with self._response_condition:
            while self._response is None:
                if not self._response_condition.wait(timeout=1.0):
                    self.get_logger().error(
                        'Timeout[%f] has expired before receving response'
                        % 1.0)
                    return
            res = self._response
            self._response = None

        if res.comm_result:
            self.get_logger().info('succesfully set value[%i] to %s'
                                   % (value, addr_name))
        else:
            self.get_logger().error('communication error when setting value[%i] to %s'
                                    % (value, addr_name))
        return res.comm_result

    def _get_response_cb(self, future):
        self._logger.info('response received')
        with self._response_condition:
            self._response = future.result()
            self._response_condition.notify_all()

    def _position(self):
        return (self._dxl_state.present_position - self._min_pos) \
             * self.position_per_tick \
             + self._min_position

    def _effort(self):
        return self._dxl_state.present_current * self.effort_per_tick

    def _is_moving(self):
        return self._dxl_state.present_velocity != 0

    def _reached_goal(self):
        return (not self._is_moving()) and \
               abs(self._dynamixel_state.present_position - self._goal_pos) <= 1

    def _stalled(self):
        return (not self._is_moving()) and \
               (self.get_clock().now() - self._last_movement_time >
                self._stall_timeout)

    def _state_values(self):
        return self._position(), self._effort(), \
               self._stalled(),  self._reached_goal()

    @property
    def position_per_tick(self):
        return (self._max_position - self._min_position) \
             / (self._max_pos      - self._min_pos)

    @property
    def effort_per_tick(self):
        return self._max_effort / self._max_cur

def main():
    rclpy.init(args=sys.argv)

    try:
        controller = PrecisionGripperController('precision_gripper_controller')
        executor   = MultiThreadedExecutor(num_threads=4)
        executor.add_node(controller)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == '__main__':
    main()
