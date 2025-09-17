#!/usr/bin/env python3
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2023, National Institute of Advanced Industrial Science and Technology (AIST)
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
# Author: Toshio Ueshiba (t.ueshiba@aist.go.jp)
#
import rclpy, sys, time, threading
from rclpy.node            import Node
from rclpy.executors       import (ExternalShutdownException,
                                   SingleThreadedExecutor,
                                   MultiThreadedExecutor)
from rclpy.callback_groups import (MutuallyExclusiveCallbackGroup,
                                   ReentrantCallbackGroup)
from rclpy.action          import ActionServer, GoalResponse, CancelResponse
from aist_msgs.action      import SuctionToolCommand
from ur_msgs.msg           import IOStates
from ur_msgs.srv           import SetIO
from std_msgs.msg          import Bool
from sensor_msgs.msg       import JointState

#########################################################################
#  class SuctionToolController                                          #
#########################################################################
class SuctionToolController(Node):
    def __init__(self, name):
        super().__init__(name)

        driver_ns = self.declare_parameter('driver_ns',
                                           'b_bot_io_and_status_controller') \
                        .value

        # Initialize ur_control table
        self._in_port    = self.declare_parameter('digital_in_port', -1).value
        self._suck_port  = self.declare_parameter('digital_out_port_suck',
                                                  -1).value
        self._blow_port  = self.declare_parameter('digital_out_port_blow',
                                                  -1).value
        self._joint_name = self.declare_parameter('joint_name', '').value

        # Create a subscriber for I/O states.
        self._suctioned_condition = threading.Condition()
        self._suctioned           = None
        self._io_states_cbg       = MutuallyExclusiveCallbackGroup()
        self._io_states_sub       = self.create_subscription(
                                        IOStates, driver_ns + '/io_states',
                                        self._io_states_cb, 10,
                                        callback_group=self._io_states_cbg)

        # Create a publisher for suction state.
        self._suctioned_pub = self.create_publisher(Bool, '~/suctioned', 1)
        self._suctioned     = False

        # Create a publisher for JointState.
        if self._joint_name != '':
            self._joint_state_pub = self.create_publisher('/joint_states',
                                                          JointState, 1)
            self._min_pos     = self.decalre_parameter('min_position').value
            self._max_pos     = self.declare_parameter('max_position').value
            self._current_pos = self._min_pos

        # Create a service client for setting digital I/O.
        #rospy.wait_for_service(driver_ns + '/set_io')
        self._set_io_cbg = MutuallyExclusiveCallbackGroup()
        self._set_io     = self.create_client(SetIO, driver_ns + '/set_io',
                                              callback_group=self._set_io_cbg)
        # if not self._set_io.wait_for_service(timeout_sec=10.0):
        #     raise TimeoutError('failed to connect server[SetIO]')

        # Create an action server for processing commands to suction tools.
        self._goal_lock       = threading.Lock()
        self._goal_handle     = None
        self._suction_cmd_cbg = MutuallyExclusiveCallbackGroup()
        self._suction_cmd_srv \
            = ActionServer(self, SuctionToolCommand, '~/command',
                           execute_callback=self._execute_cb,
                           goal_callback=self._goal_cb,
                           handle_accepted_callback=self._handle_accepted_cb,
                           cancel_callback=self._cancel_cb,
                           callback_group=self._suction_cmd_cbg)

        self.get_logger().info('controller started')

    def _io_states_cb(self, io_states):
        # Publish joint_states.
        if self._joint_name != '':
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name         = [self._joint_name]
            joint_state.position     = [self._current_pos]
            self._joint_state_pub.publish(joint_state)

        # Find the state of my IN port and publish its digital IN state
        # as a flag describing the suctioned state.
        if self._in_port >= 0:
            in_state = next(filter(lambda in_state:
                                   in_state.pin == self._in_port,
                                   io_states.digital_in_states), None)
            if in_state is None:
                self.get_logger().error(
                    'no digital IN state found at port[%d]' % self._in_port)
                return
            # Publish suction state.
            suctioned = in_state.state
            self._suctioned_pub.publish(Bool(data=suctioned))
        else:
            suctioned = False

        # Notify the execute callback that the suction state is available.
        with self._suctioned_condition:
            self._suctioned = suctioned
            self._suctioned_condition.notify_all()

    def _goal_cb(self, goal_request):
        self.get_logger().info('goal received[on=%d]' % goal_request.suck)
        return GoalResponse.ACCEPT

    def _handle_accepted_cb(self, goal_handle):
        with self._goal_lock:
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger.error('Previous goal ABORTED')
                self._goal_handle.abort()
            self._goal_handle = goal_handle  # Keep the new goal handle.
        goal_handle.execute()

    def _cancel_cb(self):
        self.get_logger().warn('cancel request reveived')
        self._set_out_port(self._blow_port, False)  # If blowing, stop it.
        return CancelResponse.ACCPET

    def _execute_cb(self, goal_handle):
        # Set states of suck and blow ports.
        self._set_out_port(self._suck_port, goal_handle.request.suck)
        self._set_out_port(self._blow_port, not goal_handle.request.suck)

        # Initialize the suction state with the desired value.
        suctioned  = goal_handle.request.suck

        start_time = self.get_clock().now()
        while goal_handle.is_active:
            # Wait for the suction state being available.
            with self._suctioned_condition:
                while self._suctioned is None:
                    if not self._suctioned_condition.wait(timeout=1.0):
                        goal_handle.abort()
                        self.get_logger().error(
                            'goal ABORTED[no incoming IO states]')
                        return SuctionToolCommand.Result(suctioned=False)
                # If no IN ports is watched, update the suction state
                # with the latest value.
                if self._in_port >=0:
                    suctioned = self._suctioned
                self._suctioned = None    # Wait for the next suction state.

            # If joint name is specified, set its value according
            # to the desired suction state.
            if self._joint_name != '':
                self._current_pos \
                    = self._max_pos if goal_handle.request.suck else \
                      self._min_pos

            goal_handle.publish_feedback(
                SuctionToolCommand.Feedback(suctioned=suctioned))

            # If the IN port has not reached the target state,
            # reset start time.
            if suctioned != self.goal_handle.request.suck:
                start_time = self.get_clock().now()

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().warn('goal CANCELED')
            # Check whether the target state has remained for min_period.
            #  - If min_period is zero, the goal succeeds immediately.
            #  - If min_period is negative, the goal never succeeds
            #    and should be terminated by a cancel request.
            elif self.goal_handle.request.min_period >= rclpy.Duration(0) and \
                 self.get_clock().now() - start_time \
                 >= goal_handle.request.min_period:
                self._set_out_port(self._blow_port, False)  # Stop blowing.
                goal_handle.succeed()
                self.logger().info('goal SUCCEEDED: suctioned')

        return SuctionToolCommand.Result(suctioned=suctioned)

    def _set_out_port(self, port, state):
        if port < 0:        # blow_port may be None
            return

        req = SetIO.Request()
        req.fun   = SetIO.Request.FUN_SET_DIGITAL_OUT
        req.pin   = port
        req.state = SetIO.Request.STATE_ON if state else \
                    SetIO.Request.STATE_OFF
        future = self._set_io.call_async(req)

        while not future.done():
            time.sleep(0.01)
        self.logger().info('set OUT port[%d] to state[%f]' % (port, state))
        return future.result()


#########################################################################
#  Entry point                                                          #
#########################################################################
if __name__ == '__main__':
    rclpy.init(args=sys.argv)

    try:
        controller = SuctionToolController('suction_tool_controller')
        executor   = MultiThreadedExecutor(num_threads=4)
        executor.add_node(controller)
        executor.spin()
    except TimeoutError as e:
        print(e)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
