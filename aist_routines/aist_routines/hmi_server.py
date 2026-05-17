#!/usr/bin/env python3
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
from rclpy.node            import Node
from rclpy.experimental    import EventsExecutor
from rclpy.action          import GoalResponse, CancelResponse
from rclpy.callback_groups import (MutuallyExclusiveCallbackGroup,
                                   ReentrantCallbackGroup)
from aist_msgs.msg         import RequestHelp as RequestHelpMsg, Pointing
from aist_msgs.action      import RequestHelp
from geometry_msgs.msg     import (QuaternionStamped, PoseStamped,
                                   PointStamped, Vector3Stamped,
                                   Point, Quaternion, Vector3)
from task_wrappers         import ActionServer

######################################################################
#  class HMIServer                                                   #
######################################################################
class HMIServer(Node):
    _Pointing = ('NO_RES', 'SWEEP_RES', 'RECAPTURE_RES')
    _NoReq    = RequestHelp(robot_name='unknown_robot_name',
                            item_id='unknown_part_ID',
                            request=RequestHelp.NO_REQ,
                            message='')

    def __init__(self, name):
        super().__init__(name)

        # RequestHelp message publishing stuffs: ROS -> Unity
        period = self.declare_parameter('period', 0.100).value
        self._request_help_pub = self.create_publisher(RequestHelpMsg,
                                                       '/help', 10)
        self._timer_cbg        = MutuallyExclusiveCallbackGroup()
        self._timer            = self.create_timer(
                                     period, self._timer_cb,
                                     callback_group=self._timer_cbg)

        # Pointing message subscription stuffs: ROS <- Unity
        self._pointing      = None
        self._pointing_cond = threading.Condition()
        self._pointing_sub  = self.create_subscription(Pointing, '/pointing',
                                                       self._pointing_cb, 3)

        # RequestHelp action server stuffs
        self._goal_handle      = None
        self._goal_handle_lock = threading.Lock()
        self._request_help_cbg = ReentrantCallbackGroup()
        self._request_help_srv = ActionServer(
                                     self, RequestHelp, '~/request_help',
                                     self._execute_cb,
                                     callback_group=self._request_help_cbg)
        self.get_logger().info('started')

    def _timer_cb(self):
        """ Publish messages requesting for help toward the remote operator.

        If the RequestHelp action server is active, publish message
        of RequestHelp type in the goal request. Otherwise, publish message
        with NO_REQ reqeust field.
        """
        with self._goal_handle_lock:
            req = self._goal_handle.request.request \
                  if self._goal_handle is not None else HMIServer._NoReq
        req.pose.header.stamp = self.get_clock().now().to_msg()
        self._request_help_pub.publish(req)

    def _pointing_cb(self, pointing: Pointing):
        """ Subscribe error recovery command messages from the remote operator.

        Reception of the message is notified to the execution callback
        of the action server.
        """
        pointing.header.stamp = self.get_clock().now().to_msg()

        with self._pointing_cond:
            self._pointing = pointing
            self._pointing_cond.notify_all()

    def _execute_cb(self, goal_handle):
        with self._goal_handle_lock:
            self._goal_handle = goal_handle

        while goal_handle.is_active:
            # Get subscribed pointing message from VR side.
            with self._pointing_cond:
                if not self._pointing_cond.wait_for(lambda:
                                                    self._pointing is not None,
                                                    1.0):
                    goal_handle.abort()
                    self.get_logger().error('timeout expired while waiting for pointing message from the remote operator')
                    pointing = Pointing(pointing_state=Pointing.NO_RES)
                    break
                pointing = self._pointing
                self._pointing = None

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().warn('goal CANCELED')
                break

            if pointing.pointing_state != Pointing.NO_RES:
                goal_handle.succeed()
                self.get_logger().info('goal SUCCEEDED[%s: pos=(%f %f %f)]'
                                       % (HMIServer \
                                          ._Pointing[pointing.pointing_state],
                                          pointing.point.x, pointing.point.y,
                                          pointing.point.z))
                break

            goal_handle.publish_feedback(
                RequestHelpAction.Feedback(response=pointing))

        with self._goal_handle_lock:
            self._goal_handle = None
        return RequestHelpAction.Result(response=pointing)

def main():
    try:
        rclpy.init(args=sys.argv)
        hmi_server = HMIServer('hmi_server')
        executor   = EventsExecutor()
        executor.add_node(hmi_server)
        executor.spin()
        hmi_srver.destroy_node()

    except Exception as e:
        print('*** Terminate the node due to exception: %s' % e)
    finally:
        rclpy.shutdown()

#########################################################################
#  entry point                                                          #
#########################################################################
if __name__ == '__main__':
    main()
