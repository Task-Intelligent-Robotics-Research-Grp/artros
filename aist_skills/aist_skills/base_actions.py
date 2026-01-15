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
from enum                  import Enum, auto
from typing                import TypeVar, Generic
from collection            import queue
from rclpy.node            import Node
from rclpy.callback_groups import (MutuallyExclusiveCallbackGroup,
                                   ReentrantCallbackGroup)
from rclpy.duration        import Duration
from rclpy.time            import Time
from rclpy.action.server   import (ActionServer, ServerGoalHandle,
                                   GoalResponse, CancelResponse)
from rclpy.action.client   import ActionClient, ClientGoalHandle

class GoalProcessingPolicy(Enum):
    SINGLE = auto()
    QUEUED = auto()
    MULTI  = auto()

class ServerGoalHandleBuffer(object):
    def __init__(self, action_name, logger):
        super().__init__()
        self._action_name = action_name
        self._logger      = logger
        self._lock        = threading.Lock()
        self._goal_handle = None

    def append(self, goal_handle):
        with self._lock:
            if self._goal_handle is not None:
                self._goal_handle.abort()
                self._logger.warn('current goal[%d@%s] ABORTED'
                                  % (self._goal_handle.goal_id,
                                     self._action_name))
            goal_handle.execute()
            self._logger.info('new goal[%d@%s] started'
                              % (goal_handle.goal_id, self._action_name))
            self._goal_handle = goal_handle

    def remove(self, goal_handle):
        self._logger.info('current goal[%d@%s] finished'
                          % (goal_handle.goal_id, self._action_name))
        with self._lock:
            self._goal_handle = None

class ServerGoalHandleQueue(object):
    def __init__(self, action_name, logger):
        super().__init__()
        self._action_name = action_name
        self._logger      = logger
        self._lock        = threading.Lock()
        self._deque       = deque()

    def append(self, goal_handle):
        with self._lock:
            if len(self._deque) == 0:
                goal_handle.execute()
                self._logger.info('new goal[%d@%s] started'
                                  % (goal_handle.goal_id, self._action_name))
            else:
                self._logger.info('new goal[%d@%s] enqueued'
                                  % (goal_handle.goal_id, self._action_name))
            self._deque.append(goal_handle)

    def remove(self, goal_handle):
        self._logger.info('current goal[%d@%s] finished'
                          % (goal_handle.goal_id, self._action_name))
        with self._lock:
            self._deque.remove(goal_handle)
            if len(self._deque) > 0:
                self._deque[0].execute()
                self._logger.info('suspended goal[%d@%s] started'
                                  % (self._deque[0].goal_id,
                                     self._action_name))

class ServerGoalHandlePassthrough(object):
    def __init__(self, action_name, logger):
        super().__init__()
        self._action_name = action_name
        self._logger      = logger

    def append(self. goal_handle):
        goal_handle.execute()
        self._logger.info('new goal[%d@%s] started'
                          % (goal_handle.goal_id, self._action_name))

    def remove(self, goal_handle):
        self._logger.info('current goal[%d@%s] finished'
                          % (goal_handle.goal_id, self._action_name))


T = TypeVar('T')

class ServerGaolHandlesDict(Generic[T]):
    def __init__(self, action_name, logger):
        self._action_name = action_name
        self._logger      = logger
        self._dict        = dict[str, T]()

    def append(self, goal_handle):
        group_name = goal_handle.request.group_name
        if not group_name in self._dict:
            self._dict[group_name] = T(self._action_name, self._logger)
            self._dict[group_name].append(goal_handle)

    def remove(self, goal_handle):
        group_name = goal_handle.request.group_name
        self._dict[group_name].remove(goal_handle)


class ActionServerBase(object):
    def __init__(self, node, action_type, action_name, user_execute_callback,
                 goal_processing_policy=GoalProcessingPolicy.SINGLE,
                 grouping=False)
        super().__init__()

        self._node = node

        # Server settings
        if goal_processing_policy == GoalProcessingPolicy.SINGLE:
            if grouping:
                self._server_goal_handles \
                    = ServerGoalHandlesDict[ServerGoalHandleBuffer](
                        action_name, self.logger)
            else:
                self._server_goal_handles = ServerGoalHandleBuffer(action_name,
                                                                   self.logger)
        elif goal_processing_policy == GoalProcessingPolicy.QUEUED:
            if grouping:
                self._server_goal_handles \
                    = ServerGoalHandlesDict[ServerGoalHandleQueue](action_name,
                                                                   self.logger)
            else:
                self._server_goal_handles = ServerGoalHandleQueue(action_name,
                                                                   self.logger)
        else:
            self._server_goal_handles \
                = ServerGoalHandlePassthrough(action_name, self.logger)

        self._user_execute_callback = user_execute_callback
        self._cbg = ReentrantCallbackGroup()
        self._srv = ActionServer(node, action_type, action_name,
                                 callback_group=self._cbg,
                                 execute_callback=self._execute_callback,
                                 goal_callback=self._goal_callback,
                                 handle_accepted_callback=self._handle_accepted_callback,
                                 cancel_callback=self._cancel_callback)

        self.logger.info('Action server[%s] started' % action_name)

    @property
    def node(self):
        return self._node

    @property
    def logger(self):
        return self._node.get_logger()

    def _goal_callback(self, qoal_request):
        self.logger.info('new goal received')
        return GoalResponse.ACCEPT

    def _handle_accepted_callback(self, goal_handle):
        self._goal_handle_buffer.append(goal_handle)

    def _cancel_callback(self, goal_handle):
        self.logger.warn('Received cancel request')
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        try:
            self._user_execute_callback(goal_handle)
        finally:
            self._goal_handle_buffer.remove(goal_handle)


class ActionClientBase(object):
    def __init__(self, node action_type, action_name):
        super().__init__()

        self._node = node

        # Client settings
        self._cbg  = MutuallyExclusiveCallbackGroup()
        self._clnt = ActionClient(node, action_type, action_name,
                                  callback_group=self._cbg)
        self._clnt.wait_for_server()

        self.logger.info('Action clinet[%s] started' % action_name)

    def send_goal(self, goal):
        pass
