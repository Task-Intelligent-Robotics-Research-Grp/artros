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
import rclpy, threading, copy
from rclpy.duration  import Duration
from rclpy.action    import ActionClient
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal

######################################################################
#  class SimpleActionClient                                          #
######################################################################
class SimpleActionClient(object):
    """
    Gripper client of control_msg/GripperCommandAction type.
    """
    def __init__(self, node, action_type, action_ns, callback_group=None):
        super().__init__()

        self._logger      = node.get_logger()
        self._goal_handle = None        # Should be kept for canceling
        self._result      = None
        self._condition   = threading.Condition()
        self._client      = ActionClient(node, action_type, action_ns,
                                         callback_group=callback_group)

    @property
    def logger(self):
        return self._logger

    def wait_for_server(self, timeout_sec=None):
        return self._client.wait_for_server(timeout_sec)

    def send_goal(self, goal, feedback_cb=None, timeout=None):
        def _goal_response_cb(future):
            def _done_cb(future):
                self._goal_handle = None
                with self._condition:
                    self._result = (future.result().status,
                                    future.result().result)
                    self._condition.notify_all()

            self._goal_handle = future.result()
            if not self._goal_handle.accepted:
                self._logger.error('goal REJECTED')
                with self._condition:
                    self._result = (-1, None)
                    self._condition.notify_all()
                    return
            self._logger.info('goal ACCEPTED')
            self._goal_handle.get_result_async().add_done_callback(_done_cb)

        self._result = None
        self._client.send_goal_async(goal, feedback_callback=feedback_cb) \
                    .add_done_callback(_goal_response_cb)
        if timeout is None:
            return
        return self.wait(timeout)

    def wait(self, timeout=Duration()):
        timeout_sec = None if timeout == Duration() else \
                      timeout.nanoseconds*1.0e-9

        with self._condition:
            if not self._condition.wait_for(lambda: self._result is not None,
                                            timeout_sec):
                self._logger.error('Timeout[%fsec] has expired' % timeout_sec)
                return GoalStatus.STATUS_UNKNOWN, None
            result = copy.deepcopy(self._result)
        if result[0] == GoalStatus.STATUS_SUCCEEDED:
            self._logger.info('goal SUCCEEDED')
        elif result[0] == GoalStatus.STATUS_CANCELED:
            self._logger.warn('goal CANCELED')
        elif result[0] == GoalStatus.STATUS_ABORTED:
            self._logger.error('goal ABORTED')
        else:
            self._logger.error('goal FAILED[status=%d]' % result[0])
        return result

    def cancel(self):
        print('### cancel() called!')

        def _cancel_response_cb(future):
            cancel_response = future.result()
            if cancel_response.return_code != CancelGoal.Response.ERROR_NONE:
                self._logger.error('request for cancelling goal REJECTED[error_code=%d]'
                                   % cancel_response.return_code)
                with self._condition:
                    self._result = (-1, None)
                    self._condition.notify_all()
                    return
            self._logger.info('request for cancelling goal ACCEPTED')

        if self._goal_handle is None:
            self._logger.warn('no active goals')
            return
        self._goal_handle.cancel_goal_async() \
                         .add_done_callback(_cancel_response_cb)
