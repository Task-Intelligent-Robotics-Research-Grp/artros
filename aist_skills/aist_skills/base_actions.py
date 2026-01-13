#  BSD 3-Clause License
#
#  Copyright (c) 2026, National Institute of Advanced Industrial Science
#  and Technology(AIST)
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.
#
#  3. Neither the name of the copyright holder nor the names of its
#     contributors may be used to endorse or promote products derived from
#     this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
#  OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
#  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
#  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
#  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
#  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
#  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#  Author: Toshio Ueshiba (t.ueshiba@aist.go.jp)
#
import queue
from rclpy.node            import Node
from rclpy.callback_groups import (MutuallyExclusiveCallbackGroup,
                                   ReentrantCallbackGroup)
from rclpy.duration        import Duration
from rclpy.time            import Time
from rclpy.action.server   import (ActionServer, ServerGoalHandle,
                                   GoalResponse, CancelResponse)
from rclpy.action.client   import ActionClient, ClientGoalHandle


class BaseAction(object):
    class ServerGoalHandleBuffer(object):
        def __init__():
            super().__init__()
            self._goal_handle = None

        def push(self, goal_handle):
            if self._goal_handle is not None:
                self._goal_handle.abort()
            self._goal_handle = goal_handle

        def pop(self):
            self._goal_handle = None

        def peek(self):
            return self._goal_handle

    class ServerGoalHandleQueue(object):
        def __init__():
            super().__init__()
            self._queue = deque()

        def push(self, goal_handle):
            self._queue.append(goal_handle)

        def pop(self):
            self._queue.popleft()

        def peek(self):
            return self._queue[0]

    def __init__(self, node, action_type, action_name, concurrent=False):
        super().__init__()

        self._node = node
        self._lock = threading.Lock()
        self._cbg  = ReentrantCallbackGroup() if concurrent else \
                     MutuallyExclusiveCallbackGroup()
        self._srv  = ActionServer(node, action_type, action_name,
                                  callback_group=self._cbg,
                                  execute_callback=self._execute_callback,
                                  goal_callback=self._goal_callback,
                                  handle_accepted_callback=self._handle_accepted_callback,
                                  cancel_callback=self._cancel_callback)

    @property
    def node(self):
        return self._node

    @property
    def logger(self):
        return self._node.get_logger()

    # Server stuffs
    def _goal_callback(self, qoal_request):
        self.logger.info('goal request received')
        return GoalResponse.ACCEPT

    def _handle_accepted_callback(self, goal_handle):
        with self._lock:
            self._goal_handle_buffer.push(goal_handle)
            self._goal_handle_bugger.peek().execute()

    def _execute_callback(self, goal_handle):
        try:
            pass
        finally:
            try:
                with self._lock:
                    self._goal_handle_buffer.pop()
                    self._goal_handle_buffer.peek().execute()
            except:
                pass
