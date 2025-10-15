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
import rclpy, time
from rclpy.node               import Node
from rclpy.duration           import Duration
from rclpy.callback_groups    import MutuallyExclusiveCallbackGroup
from rclpy.action             import ActionClient
from action_msgs.msg          import GoalStatus
from control_msgs.action      import GripperCommand
from control_msgs.msg         import GripperCommand as GripperCommandMsg
from std_msgs.msg             import Bool
from aist_msgs.action         import SuctionToolCommand
from aist_msgs.action         import ScrewToolCommand
from aist_robotiq_msgs.srv    import SetVelocity
from aist_robotiq_msgs.action import EPickCommand
from aist_robotiq_msgs.msg    import EPickCommand as EPickCommandMsg

######################################################################
#  class GripperClient                                               #
######################################################################
class GripperClient(object):
    def __init__(self, node, name, base_link=None, tip_link=None):
        super().__init__()

        self._clock      = node.get_clock()
        self._logger     = node.get_logger()
        self._name       = name
        self._base_link  = base_link if base_link else name + '_base_link'
        self._tip_link   = tip_link if tip_link else name + '_tip_link'
        self._properties = {}

    @staticmethod
    def create(node, name, type_name, props):
        ClientClass = globals().get(type_name)
        if ClientClass is None:
            raise RuntimeError(
                'unknown type[%s] of the gripper[%s]' % (type_name, name))
        try:
            return ClientClass(node, name, **props)
        except RuntimeError as e:
            node.get_logger().warn(
                'create a dummy controller for gripper[%s] because %s' % (name, e))
            return ClientClass.simulated(node, name, **props)

    @property
    def clock(self):
        return self._clock

    @property
    def logger(self):
        return self._logger

    @property
    def name(self):
        return self._name

    @property
    def base_link(self):
        return self._base_link

    @property
    def tip_link(self):
        return self._tip_link

    @property
    def properties(self):
        """
        Return a dictionary of grippaer properties
        @return a dictionary of grippaer properties with string keys
        """
        return self._properties

    @properties.setter
    def properties(self, props):
        """
        Set a dictionary of grippaer properties
        @param properties a dictionary of grippaer properties with string keys
        """
        for key, value in props.items():
            self._properties[key] = value

    def pregrasp(self):
        self.release(Duration(-1))

    def grasp(self, timeout=Duration()):
        return True

    def postgrasp(self):
        self.grasp(Duration(-1))

    def release(self, timeout=Duration()):
        return True

    def move(self, position):
        return True

    def wait(self):
        return True

    def cancel(self):
        pass

######################################################################
#  class GenericGripper                                              #
######################################################################
class GenericGripper(GripperClient):
    """
    Gripper client of control_msg/GripperCommandAction type.
    """
    def __init__(self, node, name, base_link=None, tip_link=None,
                 min_position=0.0, max_position=0.1, max_effort=5.0):
        """
        Constructor
        @param node         node object
        @param name         name of the gripper
        @param min_position position when fully closed
        @param max_position position when fully opened
        @param max_effort   maximum effort applied when gripping objects
        """
        super().__init__(node, name, base_link, tip_link)

        self._feedback = GripperCommand.Feedback()
        self._client   = ActionClient(node, GripperCommand,
                                      name + '_controller/gripper_cmd')
        if not self._client.wait_for_server(timeout_sec=1.0):
            raise RuntimeError(
                'failed to establish connection to the action server[%s]' \
                % (name + '_controller/gripper_cmd'))

        self._properties = {'grasp_position':   min_position,
                            'release_position': max_position,
                            'max_effort':       max_effort}

    @staticmethod
    def simulated(node, name, base_link=None, tip_link=None,
                  min_position=0.0, max_position=0.1, max_effort=5.0):
        return GripperClient(node, name, base_link, tip_link)

    def grasp(self, timeout=Duration()):
        """
        Grasp an object with the gripper.
        Desired finger position and applied effort are specified by properties
        with 'grasp_position' and 'max_effort' keys, respectively,
        @param timeout If positive, wait timeout duration until
                       the gripper completing the movement.
                       If zero, wait forever until the completion.
                       If negative, return immediately without waiting
                       for completion.
        @return result of control_msgs/GripperCommandResult type
        """
        return self.move(self.properties['grasp_position'],
                         self.properties['max_effort'], timeout)

    def release(self, timeout=Duration()):
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
        return self.move(self.properties['release_position'], 0.0, timeout)

    def move(self, position, max_effort=0.0, timeout=Duration()):
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
        self._get_result_future = None
        self._client.send_goal_async(
             GripperCommand.Goal(
                 command=GripperCommandMsg(position=position,
                                           max_effort=max_effort)),
             feedback_callback=self._feedback_cb) \
            .add_done_callback(self._goal_response_cb)
        return self.wait(timeout)

    def wait(self, timeout=Duration()):
        """
        Wait the gripper for completing the movement.
        @param timeout If positive, wait timeout duration until
                       the gripper completing the movement.
                       If zero, wait forever until the completion.
                       If negative, return immediately without waiting
                       for completion.
        @return result of control_msgs/GripperCommandResult type
        """
        if timeout.nanoseconds < 0:
            return GripperCommand.Result(position=0, effort=0,
                                         stalled=False, reached_goal=False)

        timeout_time = self.clock.now() + timeout
        while self._get_result_future is None or \
              not self._get_result_future.done():
            if timeout.nanoseconds > 0 and \
               self.clock.now() > timeout_time:
                self.logger.error('timeout[%f] has expired before goal finished' %
                                   timeout.nanoseconds*1.0e-9)
                return GripperCommand.Result(position=self._feedback.position,
                                             effort=self._feedback.effort,
                                             stalled=self._feedback.stalled,
                                             reached_goal=self._feedback.reached_goal)
            time.sleep(0.1)
        return self._get_result_future.result().result

    def cancel(self):
        """
        Cancel the latest motion command sent to the gripper.
        """
        if self._client.get_state() in (GoalStatus.PENDING, GoalStatus.ACTIVE):
            self._client.cancel_goal()

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.logger.error('goal rejected')
            return
        self.logger.info('goal accepted')
        self._get_result_future = goal_handle.get_result_async()

    def _feedback_cb(self, feedback):
        self._feedback = feedback

######################################################################
#  class RobotiqGripper                                              #
######################################################################
class RobotiqGripper(GenericGripper):
    def __init__(self, node, name='a_bot_gripper',
                 max_effort=0.0, velocity=0.1):
        """
        Constructor
        @param prefix     string prefix for identifying a specific gripper
                          from multiple devices
        @param max_effort maximum effort applied when gripping objects
        @param velocity   desired speed when opening or closing the gripper
        """
        ns = name + '_controller'
        self._min_gap      = node.declare_parameter(ns + '/min_gap',
                                                    0.000).value
        self._max_gap      = node.declare_parameter(ns + '/max_gap',
                                                    0.085).value
        self._min_position = node.declare_parameter(ns + '/min_position',
                                                    0.81).value
        self._max_position = node.declare_parameter(ns + '/max_position',
                                                    0.00).value
        self._set_velocity = node.create_client(SetVelocity,
                                                ns + '/set_velocity')
        if self._set_velocity.wait_for_service(timeout_sec=5.0):
            node.get_logger().error(
                'failed to establish connection to the service[%s]'
                % (ns + '/set_velocity'))

        assert self._min_gap < self._max_gap
        assert self._min_position != self._max_position

        super().__init__(node, name, None, None,
                         self._min_gap, self._max_gap, max_effort)

    def move(self, gap, max_effort=0.0, timeout=Duration()):
        return super().move(self._position(gap), max_effort, timeout)

    def wait(self, timeout=Duration()):
        result = super().wait(timeout)
        result.position = self._gap(result.position)
        return result

    def _position(self, gap):
        return (gap - self._min_gap) * self._position_per_gap \
             + self._min_position

    def _gap(self, position):
        return (position - self._min_position) / self._position_per_gap \
             + self._min_gap

    @property
    def _position_per_gap(self):
        return (self._max_position - self._min_position) \
             / (self._max_gap - self._min_gap)

######################################################################
#  class PrecisionTool                                               #
######################################################################
class PrecisionTool(GenericGripper):
    def __init__(self, node, name, base_link=None, tip_link=None):
        ns = name + '_controller'
        min_position = node.declare_parameter(ns + '/min_position', 0.00).value
        max_position = node.declare_parameter(ns + '/max_position', 0.01).value
        max_effort   = node.declare_parameter(ns + '/max_effort',   0.50).value
        assert min_position < max_position

        super().__init__(node, name, base_link, tip_link,
                         min_position, max_position, max_effort)

######################################################################
#  class EPickGripper                                                #
######################################################################
class EPickGripper(GripperClient):
    """
    Gripper client of aist_robotiq/EPickCommandAction type.
    """
    def __init__(self, node, name='a_bot_gripper', advanced_mode=False,
                 grasp_pressure=-78.0, detection_pressure=-10.0,
                 release_pressure=0.0):
        """
        Constructor
        @param prefix     string prefix for identifying a specific gripper
                          from multiple devices
        """
        super().__init__(node, name)

        ns = prefix + '_controller'
        self._feedback = EPickCommand.Feedback()
        self._client   = ActionClient(node, EPickCommand,
                                      ns + '/gripper_cmd')
        if not self._client.wait_for_server(timeout_sec=1.0):
            raise RuntimeError(
                'failed to establish connection to the controller[%s]' \
                % (ns + '/gripper_cmd'))

        self._properties = {'advanced_mode':      advanced_mode,
                            'grasp_pressure':     grasp_pressure,
                            'detection_pressure': detection_pressure,
                            'release_pressure':   release_pressure}

    @staticmethod
    def simulated(node, name, advanced_mode=False,
                  grasp_pressure=-78.0, detection_pressure=-10.0,
                  release_pressure=0.0):
        return GripperClient(node, name)

    def grasp(self, timeout=Duration()):
        """
        Grasp an object with the gripper.
        Pressure applied and pressure threshold for object detection are
        specified by properties 'grasp_pressure' and 'detection_pressure',
        respectively,
        @param timeout If positive, wait timeout duration until
                       the gripper completing the grasp action.
                       If zero, wait forever until the completion.
                       If negative, return immediately without waiting
                       for completion.
        @return result of aist_robotiq/EPickCommandResult type
        """
        return self.move(self.properties['grasp_pressure'],
                         self.properties['detection_pressure'],
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
        return self.move(self.properties['release_pressure'],
                         self.properties['detection_pressure'],
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
                     advanced_mode=self.properties['advanced_mode'],
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

        timeout_time = self.clock.now() + timeout
        while self._get_result_future is None or \
              not self._get_result_future.done():
            if timeout.nanoseconds > 0 and \
               self.clock.now() > timeout_time:
                self.logger.error('Timeout[%f] has expired before goal finished'
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
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.logger.error('goal rejected')
            return
        self.logger.info('goal accepted')
        self._get_result_future = goal_handle.get_result_async()

    def _feedback_cb(self, feedback):
        self._feedback = feedback

######################################################################
#  class SuctionTool                                                 #
######################################################################
class SuctionTool(GripperClient):
    """
    Suction tool client of aist_msgs.action.SuctionToolCommand type.
    """
    def __init__(self, node, name, base_link=None, tip_link=None,
                 suck_min_period=0.5, blow_min_period=0.2):
        super().__init__(node, name, base_link, tip_link)

        self._goal_handle       = None
        self._get_result_future = None

        ns = name + '_controller'
        self._suction_cmd = ActionClient(node, SuctionToolCommand,
                                         ns + '/command')
        if not self._suction_cmd.wait_for_server(timeout_sec=1.0):
            raise RuntimeError(
                'failed to establish connection to the action server[%s]' \
                % (ns + '/command'))

        self._suctioned     = None
        self._suctioned_cbg = MutuallyExclusiveCallbackGroup()
        self._suctioned_sub = node.create_subscription(
                                  Bool, ns + '/suctioned',
                                  self._suctioned_cb, 10,
                                  callback_group=self._suctioned_cbg)
        self._properties    = {'suck_min_period': suck_min_period,
                               'blow_min_period': blow_min_period}

    @staticmethod
    def simulated(node, name, base_link=None, tip_link=None,
                  suck_min_period=0.5, blow_min_period=0.2):
        return GripperClient(node, name, base_link, tip_link)

    @property
    def properties(self):
        return self._properties

    @properties.setter
    def properties(self, properties):
        for key, value in properties.items():
            self._properties[key] = value

    def pregrasp(self):
        # Set goal.min_period to zero so that the goal succeeds immediately.
        self._send_command(True, Duration(seconds=0), Duration(seconds=-1))

    def grasp(self, timeout=Duration(seconds=-1)):
        return self._send_command(
                   True,
                   Duration(seconds=self._properties['suck_min_period']),
                   timeout)

    def postgrasp(self):
        self.pregrasp()

    def release(self, timeout=Duration(seconds=-1)):
        return self._send_command(
                   False,
                   Duration(seconds=self._properties['blow_min_period']),
                   timeout)

    def wait(self, timeout=Duration()):
        if timeout.nanoseconds < 0:  # If timeout value is negative...
            return SuctionToolCommand.Result(suctioned=self._suctioned)

        timeout_time = self.clock.now() + timeout
        while self._get_result_future is None or \
              not self._get_result_future.done():
            if timeout.nanoseconds > 0 and self.clock.now() > timeout_time:
                self.logger.error('timeout[%.1fs] has expired before goal finished'
                                   % (timeout.nanoseconds*1.0e-9))
                return SuctionToolCommand.Result(suctioned=self._suctioned)
            time.sleep(0.05)
        self.logger.info('%s' % ('suctioned' if self._suctioned else \
                                  'not suctioned'))
        return self._get_result_future.result().result

    def cancel(self):
        if not self._goal_handle:
            self.logger.warn('no active goal')
            return

        self._goal_handle.cancel_goal_async().add_done_callback(
            self._cancel_response_cb)

    def _send_command(self, suck, min_period, timeout):
        self._goal_handle = None
        self._get_result_future = None
        self._suction_cmd.send_goal_async(
            SuctionToolCommand.Goal(suck=suck,
                                    min_period=min_period.to_msg())) \
           .add_done_callback(self._goal_response_cb)
        return self.wait(timeout)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.logger.error('goal rejected')
            return
        self.logger.info('goal accepted')
        self._goal_handle = goal_handle
        self._get_result_future = goal_handle.get_result_async()

    def _cancel_response_cb(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) == 0:
            self.logger.warn('no active goals')
        else:
            self.logger.info('goal canceled')

    def _suctioned_cb(self, msg):
        self._suctioned = msg.data

######################################################################
#  class ScrewTool                                                   #
######################################################################
class ScrewTool(GripperClient):
    """
    Screw tool client of aist_msgs.action.ScrewToolCommand type.
    """
    def __init__(self, node, name, base_link=None, tip_link=None,
                 speed=1.0, retighten=True):
        super().__init__(node, name, base_link, tip_link)

        self._feedback          = ScrewToolCommand.Feedback()
        self._goal_handle       = None
        self._get_result_future = None

        ns = name + '_controller'
        self._screw_cmd_cbg = MutuallyExclusiveCallbackGroup()
        self._screw_cmd     = ActionClient(node, ScrewToolCommand,
                                           ns + '/command',
                                           callback_group=self._screw_cmd_cbg)
        self._properties    = {'speed': speed, 'retighten': retighten}

        if not self._screw_cmd.wait_for_server(timeout_sec=1.0):
            raise RuntimeError(
                'failed to establish connection to the action server[%s]' \
                % (ns + '/command'))

    @staticmethod
    def simulated(node, name, base_link=None, tip_link=None,
                  speed=1.0, retighten=True):
        return GripperClient(node, name, base_link, tip_link)

    @property
    def properties(self):
        """
        Return a dictionary of grippaer properties
        @return a dictionary of grippaer properties with string keys
        """
        return self._properties

    @properties.setter
    def properties(self, properties):
        """
        Set a dictionary of grippaer properties
        @param properties a dictionary of grippaer properties with string keys
        """
        for key, value in properties.items():
            self._properties[key] = value

    def tighten(self, timeout=Duration(seconds=-1)):
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
        return self._send_goal(self.properties['speed'],
                               self.properties['retighten'], timeout)

    def loosen(self, timeout=Duration(seconds=-1)):
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
        return self._send_goal(-self.properties['speed'], False, timeout)

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

        timeout_time = self.clock.now() + timeout
        while self._get_result_future is None or \
              not self._get_result_future.done():
            if timeout.nanoseconds > 0 and self.clock.now() > timeout_time:
                self.logger.error('timeout[%.1fs] has expired before goal finished'
                                   % (timeout.nanoseconds * 1.0e-9))
                return ScrewToolCommand.Result(stalled=False)
            time.sleep(0.05)
        return self._get_result_future.result().result

    def cancel(self):
        """
        Cancel the latest motion command sent to the gripper.
        """
        if not self._goal_handle:
            self.logger.warn('no active goals')
            return
        self._goal_handle.cancel_goal_async().add_done_callback(
            self._cancel_response_cb)

    def _send_goal(self, speed, retighten, timeout):
        self._goal_handle = None
        self._get_result_future = None
        self._screw_cmd.send_goal_async(
            ScrewToolCommand.Goal(speed=speed, retighten=retighten),
            feedback_callback=self._feedback_cb) \
           .add_done_callback(self._goal_response_cb)
        return self.wait(timeout)

    def _goal_response_cb(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.logger.error('goal rejected')
            return
        self.logger.info('goal accepted')
        self._get_result_future = self._goal_handle.get_result_async()

    def _cancel_response_cb(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) == 0:
            self.logger.warn('no active goals')
        else:
            self.logger.info('goal canceled')

    def _feedback_cb(self, feedback):
        self._feedback = feedback
