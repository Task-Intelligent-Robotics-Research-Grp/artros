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
from rclpy.node               import Node
from rclpy.duration           import Duration, Infinite
from rclpy.action             import ActionClient
from rclpy.parameter_client   import AsyncParameterClient
from rclpy.callback_groups    import MutuallyExclusiveCallbackGroup
from action_msgs.msg          import GoalStatus
from control_msgs.action      import GripperCommand
from control_msgs.msg         import GripperCommand as GripperCommandMsg
from aist_robotiq_msgs.srv    import SetVelocity
from aist_robotiq_msgs.action import SwitchMode
from aist_robotiq_msgs.action import EPickCommand
from aist_robotiq_msgs.msg    import EPickCommand as EPickCommandMsg

######################################################################
#  class GenericGripper                                              #
######################################################################
class GenericGripper(object):
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
        super().__init__()

        self._parameters  = {'grasp_position':   min_position,
                             'release_position': max_position,
                             'max_effort':       max_effort}
        self._clock       = node.get_clock()
        self._logger      = node.get_logger()
        self._feedback    = GripperCommand.Feedback()
        self._condition   = threading.Condition()
        self._result      = None
        self._goal_handle = None
        self._client      = ActionClient(node, GripperCommand, action_ns)
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

    def grasp(self, timeout=Infinite):
        """
        Grasp an object with the gripper.
        Desired finger position and applied effort are specified by parameters
        with 'grasp_position' and 'max_effort' keys, respectively,
        @param timeout If positive, wait timeout duration until
                       the gripper completing the movement.
                       If zero, wait forever until the completion.
                       If negative, return immediately without waiting
                       for completion.
        @return result of control_msgs/GripperCommandResult type
        """
        return self.move(self.parameters['grasp_position'],
                         self.parameters['max_effort'], timeout)

    def release(self, timeout=Infinite):
        """
        Release an object grasped by the gripper.
        Desired finger position is specified by a parameter
        with 'release_position' key. No effort is applied.
        @param timeout If positive, wait timeout duration until
                       the gripper completing the movement.
                       If zero, wait forever until the completion.
                       If negative, return immediately without waiting
                       for completion.
        @return result of control_msgs/GripperCommandResult type
        """
        return self.move(self.parameters['release_position'], 0.0, timeout)

    def move(self, position, max_effort=0.0, timeout=Infinite):
        """
        Move fingers to the specified position with specified effort
        @param position   finger position
        @param max_effort maximum effort to be applied
        @param timeout    If positive, wait timeout duration until
                          the gripper completing the movement.
                          If zero, wait forever until the completion.
                          If negative, return immediately without waiting
                          for completion.
        @return result of control_msgs/GripperCommandResult type
        """
        def _feedback_cb(feedback):
            self._feedback = feedback

        def _goal_response_cb(future):
            def _done_cb(future):
                with self._condition:
                    self._result = future.result()
                    self._condition.notify_all()

            self._goal_handle = future.result()
            if not self._goal_handle.accepted:
                self._logger.error('goal rejected')
                return
            self._logger.info('goal accepted')
            self._goal_handle.get_result_async().add_done_callback(_done_cb)

        self._result = None
        self._client.send_goal_async(GripperCommand.Goal(
                                         command=GripperCommandMsg(
                                             position=position,
                                             max_effort=max_effort)),
                                     feedback_callback=_feedback_cb) \
            .add_done_callback(_goal_response_cb)
        return self.wait(timeout)

    def wait(self, timeout=Infinite):
        """
        Wait the gripper for completing the movement.
        @param timeout If positive, wait timeout duration until
                       the gripper completing the movement.
                       If zero, wait forever until the completion.
                       If negative, return immediately without waiting
                       for completion.
        @return result of control_msgs/GripperCommandResult type
        """
        if timeout == Duration():
            return None
        timeout_sec = None if timeout == Infinite else \
                      timeout.nanoseconds*1.0e-9
        with self._condition:
            if not self._condition.wait_for(lambda: self._result is not None,
                                            timeout_sec):
                self._logger.error('Timeout[%f] has expired before goal finished' %
                                   timeout_sec)
                return GripperCommand.Result(position=self._feedback.position,
                                             effort=self._feedback.effort,
                                             stalled=self._feedback.stalled,
                                             reached_goal=self._feedback.reached_goal)
            if self._result.status != GoalStatus.STATUS_SUCCEEDED:
                self._logger.error('goal FAILED with status[%d]'
                                  % self._result.status)
                return None
            self._logger.info('goal SUCCEEDED[position=%f, reached_goal=%d, stalled=%d]'
                              % (self._result.result.position,
                                 self._result.result.reached_goal,
                                 self._result.result.stalled))
            return self._result.result

    def cancel(self):
        """
        Cancel the latest motion command sent to the gripper.
        """
        def _cancel_response_cb(future):
            cancel_response = future.result()
            if len(cancel_response.goals_canceling) > 0:
                self._logger.info('goal canceled')
            else:
                self._logger.error('goal failed to be canceled')

        if not self._goal_handle:
            self._logger.warn('no active goals')
            return
        self._goal_handle.cancel_goal_async() \
                         .add_done_callback(_cancel_response_cb)

######################################################################
#  class RobotiqGripper                                              #
######################################################################
class RobotiqGripper(GenericGripper):
    def __init__(self, node, prefix='a_bot_gripper_', max_effort=0.0):
        ns = prefix + 'controller'
        super().__init__(node, ns + '/gripper_cmd', max_effort=max_effort)

        # Get parameters for computing gap values from the controller.
        self._param_client = AsyncParameterClient(node, ns)

        # Create service client for setting velocity.
        self._clnt_cbg     = MutuallyExclusiveCallbackGroup()
        self._set_velocity = node.create_client(SetVelocity,
                                                ns + '/set_velocity',
                                                callback_group=self._clnt_cbg)

        # Create action client for switching mode.
        self._switch_mode_result = None
        self._switch_mode        = ActionClient(node, SwitchMode,
                                                ns + '/switch_mode')
        self._switch_mode.wait_for_server()

        self.get_controller_parameters()
        # self._logger.info('RobotiqGripper: client of %s started' % ns)

    def move(self, gap, max_effort=0.0, timeout=Infinite):
        return super().move(self._position(gap), max_effort, timeout)

    def wait(self, timeout=Infinite):
        result = super().wait(timeout)
        if result is not None:
            result.position = self._gap(result.position)
        return result

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

    def switch_mode(self, mode, timeout=Infinite):
        def _goal_response_cb(future):
            def _done_cb(future):
                with self._condition:
                    self._switch_mode_result = future.result()
                    self._condition.notify_all()

            goal_handle = future.result()
            if not goal_handle.accepted:
                self._logger.error('switch_mode goal rejected')
                return
            self._logger.info('switch_mode goal accepted')
            goal_handle.get_result_async().add_done_callback(_done_cb)

        self._switch_mode.send_goal_async(SwitchMode.Goal(mode=mode)) \
                         .add_done_callback(_goal_response_cb)
        return self.wait_switch_mode(timeout)

    def wait_switch_mode(self, timeout=Infinite):
        if timeout == Duration():
            return None

        timeout_sec = None if timeout == Infinite else \
                      timeout.nanoseconds*1.0e-9
        with self._condition:
            if not self._condition.wait_for(
                    lambda: self._switch_mode_result is not None, timeout_sec):
                self._logger.error('Timeout[%f] has expired before set_mode goal finished' %
                                   timeout_sec)
                return SwitchMode.Result(success=False)
            if self._switch_mode_result.status != GoalStatus.STATUS_SUCCEEDED:
                self._logger.error('switch_mode gaol FAILED with status[%d]'
                                   % self._switch_mode_result.status)
                return None
            self._logger.info('switch_mode goal SUCCEEDED')
            return self._switch_mode_result.result

    def _position(self, gap):
        return (gap - self._min_gap[0]) * self._position_per_gap \
             + self._min_position[0]

    def _gap(self, position):
        return (position - self._min_position[0]) / self._position_per_gap \
             + self._min_gap[0]

    @property
    def _position_per_gap(self):
        return (self._max_position[0] - self._min_position[0]) \
             / (self._max_gap[0] - self._min_gap[0])

######################################################################
#  class EPickGripper                                                #
######################################################################
class EPickGripper(object):
    """
    Gripper client of aist_robotiq/EPickCommandAction type.
    """
    def __init__(self, node, prefix='a_bot_gripper_', advanced_mode=False,
                 grasp_pressure=-78.0, detection_pressure=-10.0,
                 release_pressure=0.0):
        """
        Constructor
        @param prefix     string prefix for identifying a specific gripper
                          from multiple devices
        """
        super().__init__()

        ns = prefix + 'controller'
        self._clock    = node.get_clock()
        self._logger   = node.get_logger()
        self._feedback = EPickCommand.Feedback()
        self._client   = ActionClient(node, EPickCommand,
                                      ns + '/gripper_cmd')
        self._client.wait_for_server()

        self._parameters = {'advanced_mode':      advanced_mode,
                            'grasp_pressure':     grasp_pressure,
                            'detection_pressure': detection_pressure,
                            'release_pressure':   release_pressure}

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
                       If negative, return immediately without waiting
                       for completion.
        @return result of aist_robotiq/EPickCommandResult type
        """
        return self.move(self.parameters['grasp_pressure'],
                         self.parameters['detection_pressure'],
                         timeout)

    def release(self, timeout=Duration(seconds=-1)):
        """
        Release an object grasped by the gripper.
        Value of applied pressure is specified by a parameter
        'release_pressure' which should be non-negative.
        @param timeout If positive, wait timeout duration until
                       the gripper completing the release action.
                       If zero, wait forever until the completion.
                       If negative, return immediately without waiting
                       for completion.
        @return result of aist_robotiq/EPickCommandResult type
        """
        return self.move(self.parameters['release_pressure'],
                         self.parameters['detection_pressure'],
                         timeout)

    def move(self, max_pressure, min_pressure, timeout=Duration()):
        """
        Move fingers to the specified position with specified effort
        @param max_pressure maximum pressure value applied
        @param min_pressure minimum pressure value for object detection
        @param timeout      If positive, wait timeout duration until
                            the gripper completing the move action.
                            If zero, wait forever until the completion.
                            If negative, return immediately without waiting
                            for completion.
        @return result of aist_robotiq/EPickCommandResult type
        """
        self._get_result_future = None
        self._client.send_goal_async(
             EPickCommand.Goal(
                 command=EPickCommandMsg(
                     advanced_mode=self.parameters['advanced_mode'],
                     max_pressure=max_pressure,
                     min_pressure=min_pressure,
                     timeout=timeout.to_msg())),
             feedback_cb=self._feedback_cb) \
            .add_done_callback(self._goal_response_cb)
        return self.wait(timeout)

    def wait(self, timeout=Duration()):
        """
        Wait the gripper for completing the movement.
        @param timeout If positive, wait timeout duration until
                       the gripper completing the action.
                       If zero, wait forever until the completion.
                       If negative, return immediately without waiting
                       for completion.
        @return result of aist_robotiq/EPickCommandResult type
        """
        if timeout.nanoseconds < 0:
            return EPickCommand.Result(pressure=0.0, stalled=False)

        timeout_time = self._clock.now() + timeout
        while self._get_result_future is None or \
              not self._get_result_future.done():
            if timeout.nanoseconds > 0 and \
               self._clock.now() > timeout_time:
                self._logger.error('Timeout[%f] has expired before goal finished'
                                   % timeout.nanoseconds/1.0e9)
                return EPickCommand.Result(pressure=self._feedback.pressure,
                                           stalled=self._feedback.stalled)
            time.sleep(0.1)
        return self._get_result_future.result().result

    def cancel(self):
        """
        Cancel the latest motion command sent to the gripper.
        """
        if self._client.get_state() in (GoalStatus.PENDING, GoalStatus.ACTIVE):
            self._client.cancel_goal()

    def _goal_response_cb(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self._logger.error('goal rejected')
        else:
            self._logger.info('goal accepted')
        self._get_result_future = self._goal_handle.get_result_async()

    def _feedback_cb(self, feedback):
        self._feedback = feedback
