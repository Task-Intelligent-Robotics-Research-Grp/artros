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
import rclpy, time

from rclpy.node            import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from aist_msgs.srv         import ManageCollisionObject

#########################################################################
#  class CollisionObjectManagerClient                                   #
#########################################################################
class CollisionObjectManagerClient(object):
    def __init__(self, node,
                 server='collision_object_manager', timeout_sec=5.0):
        super().__init__()

        service_ns = server + '/manage_collision_object'
        self._logger = node.get_logger()
        self._cbg    = MutuallyExclusiveCallbackGroup()
        self._client = node.create_client(ManageCollisionObject, service_ns,
                                          callback_group=self._cbg)
        if not self._client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(
                'failed to establish connection to the service[%s]' \
                % service_ns)
        node.get_logger().info('established connection to the service[%s]'
                               % service_ns)

    def create_object(self, object_type, pose,
                      subframe='base_link', object_id=''):
        req = ManageCollisionObject.Request()
        req.op          = ManageCollisionObject.Request.CREATE_OBJECT
        req.object_type = object_type
        req.object_id   = object_id if object_id != '' else object_type
        req.subframe    = subframe
        req.frame_id    = pose.header.frame_id
        req.pose        = pose.pose
        return self._send(req).success

    def remove_object(self, object_id='', frame_id=''):
        req = ManageCollisionObject.Request()
        req.op        = ManageCollisionObject.Request.REMOVE_OBJECT
        req.object_id = object_id
        req.frame_id  = frame_id
        return self._send(req).success

    def attach_object(self, object_id, parent_link, leaf_id=''):
        req = ManageCollisionObject.Request()
        req.op        = ManageCollisionObject.Request.ATTACH_OBJECT
        req.object_id = object_id
        req.frame_id  = parent_link
        req.leaf_id   = leaf_id
        res = self._send(req)
        return res.info if res.success else None

    def detach_object(self, object_id, parent_link, leaf_id=''):
        req = ManageCollisionObject.Request()
        req.op        = ManageCollisionObject.Request.DETACH_OBJECT
        req.object_id = object_id
        req.frame_id  = parent_link
        req.leaf_id   = leaf_id
        res = self._send(req)
        return res.info if res.success else None

    def move_object(self, object_id, pose, subframe='base_link'):
        req = ManageCollisionObject.Request()
        req.op        = ManageCollisionObject.Request.MOVE_OBJECT
        req.object_id = object_id
        req.subframe  = subframe
        req.frame_id  = pose.header.frame_id
        req.pose      = pose.pose
        res = self._send(req)
        return res.info if res.success else None

    def append_touch_links(self, object_id, touch_link):
        req = ManageCollisionObject.Request()
        req.op        = ManageCollisionObject.Request.APPEND_TOUCH_LINKS
        req.object_id = object_id
        req.frame_id  = touch_link
        return self._send(req).success

    def remove_touch_links(self, object_id, untouch_link):
        req = ManageCollisionObject.Request()
        req.op        = ManageCollisionObject.Request.REMOVE_TOUCH_LINKS
        req.object_id = object_id
        req.frame_id  = untouch_link
        return self._send(req).success

    def reset_touch_links(self):
        req = ManageCollisionObject.Request()
        req.op = ManageCollisionObject.Request.RESET_TOUCH_LINKS
        return self._send(req).success

    def get_object_info(self, object_id):
        req           = ManageCollisionObject.Request()
        req.op        = ManageCollisionObject.Request.GET_OBJECT_INFO
        req.object_id = object_id
        res = self._send(req)
        return res.info if res.success else None

    def get_child_object_info(self, frame_id):
        req          = ManageCollisionObject.Request()
        req.op       = ManageCollisionObject.Request \
                      .GET_ATTACHED_CHILD_OBJECT_INFO
        req.frame_id = frame_id
        res = self._send(req)
        return res.info if res.success else None

    def allow_collision(self, object_id, frame_id):
        req           = ManageCollisionObject.Request()
        req.op        = ManageCollisionObject.Request.ALLOW_COLLISION
        req.object_id = object_id
        req.frame_id  = frame_id
        return self._send(req).success

    def disallow_collision(self, object_id, frame_id):
        req           = ManageCollisionObject.Request()
        req.op        = ManageCollisionObject.Request.DISALLOW_COLLISION
        req.object_id = object_id
        req.frame_id  = frame_id
        return self._send(req).success

    def _send(self, req):
        return self._client.call(req)
