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
"""
Clients of gripper action controller of control_msg/GripperCommandAction type.
@file   __init__.py
@author t.ueshiba@aist.go.jp
"""
import rclpy, time, threading
from rclpy.node            import Node
from rclpy.duration        import Duration
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.action          import ActionClient
from action_msgs.msg       import GoalStatus
from aist_msgs.action      import ScrewToolCommand, SuctionToolCommand

######################################################################
#  class ScrewTool                                                   #
######################################################################
class ScrewTool(object):
    """
    Screw tool client of aist_msgs.action.ScrewToolCommand type.
    """
    def __init__(self, node, action_ns, speed=1.0, retighten=True):
        """
        Constructor
        @param action_ns    namespace of action server to be connected
        """
        super().__init__()

        self._clock            = node.get_clock()
        self._logger           = node.get_logger()
        self._feedback         = ScrewToolCommand.Feedback()
        self._client_cbg       = MutuallyExclusiveCallbackGroup()
        self._result_condition = threading.Condition()
        self._result           = None
        self._client           = ActionClient(node, ScrewToolCommand,
                                              action_ns,
                                              callback_group=self._client_cbg)
        self._parameters       = {'speed':     speed,
                                  'retighten': retighten}
        self._client.wait_for_server()

    @property
    def parameters(self):
        """
        Return a dictionary of grippaer parameters
        @return a dictionary of grippaer parameters with string keys
        """
        return self._parameters

    @parameters.setter
    def parameters(self, parameters):
        """
        Set a dictionary of grippaer parameters
        @param parameters a dictionary of grippaer parameters with string keys
        """
        for key, value in parameters.items():
            self._parameters[key] = value

    def tighten(self, timeout=Duration(seconds=1)):
        """
        Tighten the screw with the tool.
        Desired speed is specified by the parameter 'speed'.
        with 'grasp_position' and 'max_effort' keys, respectively,
        @param timeout If positive, wait timeout duration until
                       the tool completing the tightening.
                       If zero, wait forever until the completion.
                       If negative, return immediately without waiting
                       for completion.
        @return result of aist_msgs.action.ScrewToolCommand.Result type
        """
        return self._send_goal(self.parameters['speed'],
                               self.parameters['retighten'], timeout)

    def loosen(self, timeout=Duration(seconds=1)):
        """
        Loosen the screw with the tool.
        Desired speed is specified by the parameter 'speed'.
        @param timeout If positive, wait timeout duration until
                       the gripper completing the movement.
                       If zero, wait forever until the completion.
                       If negative, return immediately without waiting
                       for completion.
        @return result of control_msgs/GripperCommandResult type
        """
        return self._send_goal(-self.parameters['speed'], False, timeout)

    def wait(self, timeout=Duration()):
        """
        Wait the gripper for completing the movement.
        @param timeout If positive, wait timeout duration until
                       the tool completing the tightening/loosing of the screw.
                       If zero, wait forever until the completion.
                       If negative, return immediately without waiting
                       for completion.
        @return result of control_msgs/GripperCommandResult type
        """
        if timeout.nanoseconds < 0:
            return ScrewToolCommand.Result(stalled=False)

        to = timeout.nanoseconds*1.0e-9 if timeout.nanoseconds > 0 else None
        with self._result_condition:
            while self._result is None:
                if not self._result_condition.wait(timeout=to):
                    self._logger.error(
                        'Timeout[%f] has expired before goal finished' % to)
                    return ScrewToolCommand.Result(stalled=False)
            result = self._result
            self._result = None

        return result

    def cancel(self):
        """
        Cancel the latest motion command sent to the gripper.
        """
        if not self._goal_handle:
            self._logger.warn('no active goals')
            return

        self._goal_handle.cancel_goal_async().add_done_callback(
            self._cancel_response_cb)

    def _send_goal(self, speed, retighten, timeout=Duration()):
        self._goal_handle = None
        self._client.send_goal_async(
            ScrewToolCommand.Goal(speed=speed, retighten=retighten),
            feedback_callback=self._feedback_cb) \
            .add_done_callback(self._goal_response_cb)
        return self.wait(timeout)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._logger.error('goal rejected')
            return
        self._logger.info('goal accepted')
        goal_handle.get_result_async().add_done_callback(self._get_result_cb)
        self._goal_handle = goal_handle

    def _get_result_cb(self, future):
        self._logger.info('result received')
        with self._result_condition:
            self._result = future.result()
            self._result_condition.notify_all()

    def _cancel_response_cb(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) == 0:
            self._logger.warn('no active goals')
        else:
            self._logger.info('goal canceled')

    def _feedback_cb(self, feedback):
        self._feedback = feedback

######################################################################
#  class SuctionTool                                                 #
######################################################################
class SuctionTool(object):
    """
    Suction tool client of aist_msgs.action.SuctionToolCommand type.
    """
    def __init__(self, node, action_ns):
        """
        Constructor
        @param action_ns    namespace of action server to be connected
        """
        super().__init__()

        self._clock            = node.get_clock()
        self._logger           = node.get_logger()
        self._feedback         = SuctionToolCommand.Feedback()
        self._client_cbg       = MutuallyExclusiveCallbackGroup()
        self._result_condition = threading.Condition()
        self._result           = None
        self._client           = ActionClient(node, SuctionToolCommand,
                                              action_ns,
                                              callback_group=self._client_cbg)
        self._client.wait_for_server()
