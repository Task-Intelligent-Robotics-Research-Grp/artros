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
import rclpy
from rclpy.node            import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from action_msgs.msg       import GoalStatus
from std_msgs.msg          import Bool
from aist_msgs.action      import ScrewToolCommand, SuctionToolCommand
from srv_and_action_wrappers.action_client import SimpleActionClient

######################################################################
#  class SuctionTool                                                 #
######################################################################
class SuctionTool(SimpleActionClient):
    """
    Suction tool client of aist_msgs.action.SuctionToolCommand type.
    """
    def __init__(self, node, name, suck_min_period=0.5, blow_min_period=0.2):
        """
        Constructor
        @param name    name of the suction tool
        """
        self._name    = name
        controller_ns = name + '_controller'
        self._callback_group = MutuallyExclusiveCallbackGroup()

        super().__init__(node, SuctionCommand, controller_ns + '/gripper_cmd',
                         self._callback_group)

        if not self.wait_for_server(timeout_sec=1.0):
            raise RuntimeError(
                'failed to establish connection to the actionserver[%s]' \
                % (controller_ns + '/command'))

        self._suctioned     = None
        self._suctioned_cbg = MutuallyExclusiveCallbackGroup()
        self._suctioned_sub = node.create_subscription(
                                  Bool, controller_ns + '/suctioned',
                                  self._suctioned_cb, 10,
                                  callback_group=self._suctioned_cbg)
        self._properties = {'suck_min_period': suck_min_period,
                            'blow_min_period': blow_min_period}

    @property
    def name(self):
        return self._name

    @property
    def base_link(self):
        return self._name + '/base_link'

    @property
    def tip_link(self):
        return self._name + '/tip_link'

    @property
    def properties(self):
        return self._properties

    def pregrasp(self):
        # Set goal.min_period to zero so that the goal succeeds immediately.
        self._suck_command(True, 0.0)

    def grasp(self, timeout_sec=None):
        return self._suck_command(True, self._properties['suck_min_period'],
                                  timeout_sec)

    def postgrasp(self):
        self.pregrasp()

    def release(self, timeout_sec=None):
        return self._suck_command(False, self._properties['blow_min_period'],
                                  timeout_sec)

    def wait(self, timeout_sec=None):
        status, result = super().wait(timeout_sec)
        if status == GoalStatus.STATUS_UNKNOWN:
            return (status,
                    SuctionToolCommand.Result(suctioned=self._suctioned))
        return (status, result)

    def _suck_command(self, suck, min_period, timeout_sec=None):
        return self.send_goal(SuctionToolCommand.Goal(suck=suck,
                                                      min_period=min_period),
                              timeout_sec)

    def _suctioned_cb(self, msg):
        self._suctioned = msg.data

######################################################################
#  class ScrewTool                                                   #
######################################################################
class ScrewTool(SuctionTool):
    """
    Screw tool client of aist_msgs.action.ScrewToolCommand type.
    """
    def __init__(self, node, name, suck_min_period=0.5, blow_min_period=0.2,
                 speed=1.0, retighten=True):
        """
        Constructor
        @param name   name of the screw tool
        """
        super().__init__(node, name, suck_min_period, blow_min_period)

        controller_ns = name + '_fastening_controller'
        self._screw_tool = SimpleActionClient(node, ScrewToolCommand,
                                              controller_ns + '/command',
                                              callback_group=self._callback_group)
        if not self._screw_tool.wait_for_server(timeout_sec=1.0):
            raise RuntimeError(
                'failed to establish connection to the action server[%s]' \
                % (controller_ns + '/command'))
        self._properties['speed']     = speed
        self._properties['retighten'] = retighten

    def tighten(self, timeout_sec=None):
        """
        Tighten the screw with the tool.
        Desired speed is specified by the parameter 'speed'.
        @param timeout_sec If positive, wait timeout duration until
                           the gripper completing the movement.
                           If non-positive, return immediately without waiting
                           for completion.
                           If None, wait forever until the completion.
        @return (status, result) of
                (int, aist_msgs/action/ScrewToolCommand.Result) type
        """
        return self._screw_command(self.properties['speed'],
                                   self.properties['retighten'], timeout_sec)

    def loosen(self, timeout_sec=None):
        """
        Loosen the screw with the tool.
        Desired speed is specified by the parameter 'speed'.
        @param timeout_sec If positive, wait timeout duration until
                           the gripper completing the movement.
                           If non-positive, return immediately without waiting
                           for completion.
                           If None, wait forever until the completion.
        @return (status, result) of
                (int, aist_msgs/action/ScrewToolCommand.Result) type
        """
        return self._screw_command(-self.properties['speed'], False,
                                   timeout_sec)

    def pregrasp(self):
        self._screw_command(self.properties['speed'], False, 0.0)
        super().pregrasp()

    def grasp(self, timeout_sec=None):
        self._screw_command(self.properties['speed'], False, None)
        status, result = super().grasp(timeout_sec)
        if status == GoalStatus.STATUS_SUCCEEDED and result.suctioned:
            self._screw_tool.cancel()
        return status, result

    def postgrasp(self):
        self._screw_tool.cancel()
        super().postgrasp()

    def wait(self, timeout_sec=None):
        return self._screw_tool.wait(timeout_sec)

    def cancel(self):
        """
        Cancel the latest motion command sent to the gripper.
        """
        if not self._goal_handle:
            self._logger.warn('no active goals')
            return
        self._goal_handle.cancel_goal_async().add_done_callback(
            self._cancel_response_cb)

    def _screw_command(self, speed, retighten, timeout_sec):
        return self._screw_tool.send_goal(ScrewToolCommand.Goal(
                                            speed=speed, retighten=retighten),
                                          None, timeout_sec)
