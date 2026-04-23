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
import rclpy, threading
from rclpy.duration           import Duration
from rclpy.parameter_client   import AsyncParameterClient
from rclpy.callback_groups    import MutuallyExclusiveCallbackGroup
from action_msgs.msg          import GoalStatus
from control_msgs.action      import GripperCommand
from control_msgs.msg         import GripperCommand as GripperCommandMsg
from aist_robotiq_msgs.srv    import SetVelocity
from aist_robotiq_msgs.action import SetMode
from aist_robotiq_msgs.action import SuctionCommand
from aist_robotiq_msgs.msg    import SuctionCommand as SuctionCommandMsg
from .simple_action_client    import SimpleActionClient

######################################################################
#  class GenericGripper                                              #
######################################################################
class GenericGripper(SimpleActionClient):
    """
    Gripper client of control_msg/GripperCommandAction type.
    """
    def __init__(self, node, action_ns,
                 min_position=0.0, max_position=0.1, max_effort=5.0):
        """
        Constructor
        @param action_ns    namespace of action server to be connected
        @param min_position position when fully closed
        @param max_position position when fully opened
        @param max_effort   maximum effort applied when gripping objects
        """
        self._callback_group = MutuallyExclusiveCallbackGroup()
        super().__init__(node, GripperCommand, action_ns, self._callback_group)
        self.wait_for_server()

        self._parameters  = {'grasp_position':   min_position,
                             'release_position': max_position,
                             'max_effort':       max_effort}

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

    def grasp(self, timeout=Duration()):
        """
        Grasp an object with the gripper.
        Desired finger position and applied effort are specified by parameters
        with 'grasp_position' and 'max_effort' keys, respectively,
        @param timeout If positive, wait timeout duration until
                       the gripper completing the movement.
                       If zero, wait forever until the completion.
                       If None, return immediately without waiting
                       for completion.
        @return (status, result) of
                (int, control_msgs/action/GripperCommand.Result) type
        """
        return self.move(self.parameters['grasp_position'],
                         self.parameters['max_effort'], timeout)

    def release(self, timeout=Duration()):
        """
        Release an object grasped by the gripper.
        Desired finger position is specified by a parameter
        with 'release_position' key. No effort is applied.
        @param timeout If positive, wait timeout duration until
                       the gripper completing the movement.
                       If zero, wait forever until the completion.
                       If None, return immediately without waiting
                       for completion.
        @return (status, result) of
                (int, control_msgs/action/GripperCommand.Result) type
        """
        return self.move(self.parameters['release_position'], 0.0, timeout)

    def move(self, position, max_effort=0.0, timeout=Duration()):
        """
        Move fingers to the specified position with specified effort
        @param position   finger position
        @param max_effort maximum effort to be applied
        @param timeout    If positive, wait timeout duration until
                          the gripper completing the movement.
                          If zero, wait forever until the completion.
                          If None, return immediately without waiting
                          for completion.
        @return (status, result) of
                (int, control_msgs/action/GripperCommand.Result) type
        """
        return self.send_goal(GripperCommand.Goal(
                                  command=GripperCommandMsg(
                                      position=position,
                                      max_effort=max_effort)),
                              timeout=timeout)

######################################################################
#  class RobotiqGripper                                              #
######################################################################
class RobotiqGripper(GenericGripper):
    def __init__(self, node, prefix='a_bot_gripper_', max_effort=0.0):
        ns = prefix + 'controller'
        super().__init__(node, ns + '/gripper_cmd', max_effort=max_effort)

        # Create service client for setting velocity.
        self._set_velocity \
            = node.create_client(SetVelocity, ns + '/set_velocity',
                                 callback_group=self._callback_group)

        # Get parameters for computing gap values from the controller.
        self._param_client = AsyncParameterClient(node, ns)
        self.get_controller_parameters()

        # Create action client for switching mode.
        self._mode = SetMode.Goal.BASIC
        self._individual_control_fingers = True
        self._individual_control_scissor = True
        self._set_mode = SimpleActionClient(node, SetMode, ns + '/set_mode',
                                            self._callback_group)
        self.wait_for_server()

        # self._logger.info('RobotiqGripper: client of %s started' % ns)

    def move(self, gap, max_effort=0.0, timeout=Duration()):
        return super().move(self._position(gap), max_effort, timeout)

    def wait(self, timeout=Duration()):
        status, result = super().wait(timeout)
        if result is not None:
            result.position = self._gap(result.position)
        return status, result

    def get_controller_parameters(self):
        def _get_parameters_cb(future):
            values = future.result().values

            self._min_gap      = values[0].double_array_value
            self._max_gap      = values[1].double_array_value
            self._min_position = values[2].double_array_value
            self._max_position = values[3].double_array_value
            self.parameters = {'grasp_position':   self._min_gap[0],
                               'release_position': self._max_gap[0]}

        self._param_client.get_parameters(['min_gap', 'max_gap',
                                           'min_position', 'max_position'],
                                          _get_parameters_cb)

    def set_velocity(self, velocity):
        self._set_velocity.call(SetVelocity.Request(velocity=velocity)).success

    def set_mode(self, mode, individual_control_fingers=False,
                 individual_control_scissor=False):
        status, result \
            = self._set_mode.send_goal(
                  SetMode.Goal(
                      mode=mode,
                      individual_control_fingers=individual_control_fingers,
                      individual_control_scissor=individual_control_scissor),
                  timeout=Duration())
        if status == GoalStatus.STATUS_SUCCEEDED and result.success:
            self._mode = mode
            self._individual_control_fingers = individual_control_fingers
            self._individual_control_scissor = individual_control_scissor
            return True
        else:
            return False

    def _position(self, gap):
        idx = self._idx()
        return (gap - self._min_gap[idx]) * self._position_per_gap(idx) \
             + self._min_position[idx]

    def _gap(self, position):
        idx = self._idx()
        return (position - self._min_position[idx]) \
             / self._position_per_gap(idx) + self._min_gap[idx]

    def _position_per_gap(self, idx):
        return (self._max_position[idx] - self._min_position[idx]) \
             / (self._max_gap[idx]      - self._min_gap[idx])

    def _idx(self):
        return 3 if self._mode == SetMode.Goal.SCISSOR else 0

######################################################################
#  class RobotiqSuction                                              #
######################################################################
class RobotiqSuction(SimpleActionClient):
    """
    Gripper client of aist_robotiq/SuctionCommandAction type.
    """
    def __init__(self, node, prefix='a_bot_gripper_', advanced_mode=True,
                 grasp_pressure=-78.0, detection_pressure=-10.0,
                 release_pressure=0.0, grasp_timeout=0.0):
        """
        Constructor
        @param prefix     string prefix for identifying a specific gripper
        """
        ns = prefix + 'controller'
        self._callback_group = MutuallyExclusiveCallbackGroup()
        super().__init__(node, SuctionCommand, ns + '/gripper_cmd',
                         self._callback_group)
        self.wait_for_server()

        self._parameters = {'advanced_mode':      advanced_mode,
                            'grasp_pressure':     grasp_pressure,
                            'detection_pressure': detection_pressure,
                            'release_pressure':   release_pressure,
                            'grasp_timeout':      grasp_timeout}

    @property
    def callback_group(self):
        return self._callback_group

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

    def grasp(self, timeout=Duration()):
        """
        Grasp an object with the gripper.
        Pressure applied and pressure threshold for object detection are
        specified by parameters 'grasp_pressure' and 'detection_pressure',
        respectively,
        @param timeout If positive, wait timeout duration until
                       the gripper completing the grasp action.
                       If zero, wait forever until the completion.
                       If None, return immediately without waiting
                       for completion.
        @return result of aist_robotiq/SuctionCommandResult type
        """
        return self.suck(self.parameters['grasp_pressure'],
                         self.parameters['detection_pressure'],
                         timeout)

    def release(self, timeout=Duration()):
        """
        Release an object grasped by the gripper.
        Value of applied pressure is specified by a parameter
        'release_pressure' which should be non-negative.
        @param timeout If positive, wait timeout duration until
                       the gripper completing the release action.
                       If zero, wait forever until the completion.
                       If None, return immediately without waiting
                       for completion.
        @return result of aist_robotiq/SuctionCommandResult type
        """
        return self.suck(self.parameters['release_pressure'],
                         self.parameters['detection_pressure'],
                         timeout)

    def suck(self, max_pressure, min_pressure, timeout=Duration()):
        """
        Move fingers to the specified position with specified effort
        @param max_pressure maximum pressure value applied
        @param min_pressure minimum pressure value for object detection
        @param timeout      If positive, wait timeout duration until
                            the gripper completing the move action.
                            If zero, wait forever until the completion.
                            If None, return immediately without waiting
                            for completion.
        @return result of aist_robotiq/SuctionCommandResult type
        """
        return self.send_goal(
                   SuctionCommand.Goal(
                       command=SuctionCommandMsg(
                           advanced_mode=self.parameters['advanced_mode'],
                           max_pressure=max_pressure,
                           min_pressure=min_pressure,
                           timeout=Duration(
                               seconds=self.parameters['grasp_timeout']) \
                           .to_msg())),
                   timeout=timeout)
