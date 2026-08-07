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
import threading, collections
from rclpy.node                  import Node
from rclpy.callback_groups       import MutuallyExclusiveCallbackGroup
from task_wrappers.action_client import GroupedSimpleActionClient
from task_wrappers.action_server import ActionServer
from aist_msgs.action            import RequestHelp
from aist_msgs.msg               import RequestHelp as RequestHelpMsg, Pointing
from geometry_msgs.msg           import (QuaternionStamped, PoseStamped,
                                         PointStamped, Vector3Stamped,
                                         Point, Quaternion, Vector3)
from visualization_msgs.msg      import Marker

#*********************************************************************
#  class RequestHelpTaskClient                                       *
#*********************************************************************
class RequestHelpTaskClient(GroupedSimpleActionClient):
    MarkerProps = collections.namedtuple('MarkerProps', 'id, scale, color')
    _marker_props = {
        'finger' : MarkerProps(0, (0.008, 0.008, 0.008), (1.0, 0.0, 0.0, 1.0)),
        'sweep'  : MarkerProps(1, (0.006, 0.014, 0.015), (1.0, 1.0, 0.0, 1.0))
    }
    _marker_lifetime = 0

    def __init__(self, node, server_ns='request_help'):
        super().__init__(node, RequestHelp, server_ns,
                         callback_group=MutuallyExclusiveCallbackGroup(),
                         group_field='robot_name')
        self.wait_for_server()
        self._marker_pub = node.create_publisher(Marker, 'pointing_marker', 1)

    def send_goal(self, robot_name, pose, part_id, message,
                  *, timeout_sec=0.0):
        """
        Request finger direction for the specified graspability point
        and receive response.

        @type  robot_name: str
        @param robot_name: name of the robot
        @type  pose:       geometry_msgs.msg.PoseStamped
        @param pose:       pose of the graspability point to be sweeped
        @type  part_id:    str
        @param part_id:    ID for specifying part
        @type  message:    str
        @param message:    message to be displayed to the operator of VR side
        @return:           response with finger direction from VR side
        """
        req = RequestHelp.Goal()
        req.robot_name = robot_name
        req.item_id    = part_id
        req.pose       = self.node.transform_pose_to_target_frame(
                             pose, target_frame=self._ground_frame)
        req.request    = RequestHelpMsg.SWEEP_DIR_REQ
        req.message    = message
        super().send_goal(req, feedback_cb=self._feedback_cb,
                          timeout_sec=timeout_sec)

    def _publish_marker(self, marker_type, header, pos, dir=None, lifetime=15):
        """
        Publish arrow marker with specified start point and direction.

        @type  point: geometry_msgs.msg.PointStamped
        @param pos:   start point of the arrow marker
        @type  dir:   geometry_msgs.msg.Vector3
        @param dir:   direction of the arrow marker
        """
        marker_prop = RequestHelpTaskClient._marker_props[marker_type]

        marker              = Marker()
        marker.header       = header
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.ns           = 'pointing'
        marker.id           = marker_prop.id
        marker.type         = Marker.SPHERE if dir is None else Marker.ARROW
        marker.action       = Marker.ADD
        marker.scale        = Vector3(x=marker_prop.scale[0],
                                      y=marker_prop.scale[1],
                                      z=marker_prop.scale[2])
        marker.color        = ColorRGBA(r=marker_prop.color[0],
                                        g=marker_prop.color[1],
                                        b=marker_prop.color[2],
                                        a=marker_prop.color[3])
        marker.lifetime     = Duration(sec=lifetime, usec=0)
        if dir is None:
            marker.pose.position    = pos
            marker.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        else:
            t = 0.03
            marker.points.append(pos)
            marker.points.append(Point(pos.x + t*dir.x,
                                       pos.y + t*dir.y,
                                       pos.z + t*dir.z))
        self._marker_pub.publish(marker)

    def delete_markers(self):
        """ Delete all markers published by _marker_pub.
        """
        marker        = Marker()
        marker.action = Marker.DELETEALL
        marker.ns     = 'pointing'
        self._marker_pub.publish(marker)

    def _feedback_cb(self, feedback):
        self._publish_marker('finger', feedback.feedback.response.header,
                             feedback.feedback.response.point)

#*********************************************************************
#  class RequestHelpTaskServer                                       *
#*********************************************************************
class RequestHelpTaskServer(ActionServer):
    _Pointing = ('NO_RES', 'SWEEP_RES', 'RECAPTURE_RES')
    _NoReq    = RequestHelpMsg(robot_name='unknown_robot_name',
                               item_id='unknown_part_ID',
                               request=RequestHelpMsg.NO_REQ,
                               message='')

    def __init__(self, node, server_ns='request_help'):
        super().__init__(node, RequestHelp, server_ns, self._execute_cb,
                         callback_group=MutuallyExclusiveCallbackGroup(),
                         group_field='robot_name')

        # RequestHelp message publishing stuffs: ROS -> Unity
        period = node.declare_parameter('request_help.period', 0.100).value
        self._goal_handle      = None
        self._goal_handle_lock = threading.Lock()
        self._request_help_pub = node.create_publisher(RequestHelpMsg,
                                                       '/help', 10)
        self._timer = node.create_timer(
                          period, self._timer_cb,
                          callback_group=MutuallyExclusiveCallbackGroup())

        # Pointing message subscription stuffs: ROS <- Unity
        self._pointing      = None
        self._pointing_cond = threading.Condition()
        self._pointing_sub  = node.create_subscription(Pointing, '/pointing',
                                                       self._pointing_cb, 3)

    def _timer_cb(self):
        """ Publish messages requesting for help toward the remote operator.
        If the RequestHelp action server is active, publish message
        of RequestHelp type in the goal request. Otherwise, publish message
        with NO_REQ reqeust field.
        """
        with self._goal_handle_lock:
            req = self._goal_handle.request.request if self._goal_handle else \
                  RequestHelpTaskServer._NoReq
        req.pose.header.stamp = self.node.get_clock().now().to_msg()
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
                    self.logger.error('timeout expired while waiting for pointing message from the remote operator')
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
                self.logger.info('goal SUCCEEDED[%s: pos=(%f %f %f)]'
                                 % (RequestHelpTaskServer \
                                    ._Pointing[pointing.pointing_state],
                                    pointing.point.x, pointing.point.y,
                                    pointing.point.z))
                break

            goal_handle.publish_feedback(
                RequestHelpAction.Feedback(response=pointing))

        with self._goal_handle_lock:
            self._goal_handle = None
        return RequestHelpAction.Result(response=pointing)

#************************************************************************
#  class RequestHelpTask                                                *
#************************************************************************
class RequestHelpTask(RequestHelpTaskClient):
    def __init__(self, node, server_ns='request_help'):
        self._server = RequestHelpTaskServer(node, server_ns)
        super().__init__(node, server_ns)
