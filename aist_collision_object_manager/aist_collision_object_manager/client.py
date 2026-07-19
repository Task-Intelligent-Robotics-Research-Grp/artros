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
import rclpy

from rclpy.callback_groups        import MutuallyExclusiveCallbackGroup
from aist_msgs.srv                import ManageCollisionObject
from task_wrappers.service_client import ServiceClient

from typing                       import Optional
from rclpy.node                   import Node
from geometry_msgs.msg            import PoseStamped
from aist_msgs.msg                import CollisionObjectInfo

#########################################################################
#  class CollisionObjectManagerClient                                   #
#########################################################################
class CollisionObjectManagerClient(object):
    """ Client of CollisionObjectManager.
    """

    def __init__(self, node: Node, server: str='collision_object_manager',
                 timeout_sec: Optional[float]=5.0):
        """ Create a client for CollisionObjectManger server.
        Args:
          node:        The ROS node to add the client to.
          server:      Name of the server to be connected to.
          timeout_sec: Timeout time waiting for the server available.
                       Seconds to wait, if positive. Wait forever, if `None`.
        """
        super().__init__()

        service_ns = server + '/manage_collision_object'
        self._cbg    = MutuallyExclusiveCallbackGroup()
        self._client = ServiceClient(node, ManageCollisionObject, service_ns,
                                     callback_group=self._cbg)
        if not self._client.wait_for_service(timeout_sec=timeout_sec):
            raise TimeoutError('timeout expired before conneted to service[%s]'
                               % service_ns)

    def create_object(self, object_type: str, pose: PoseStamped,
                      subframe: str='base_link', object_id: str='',
                      *, timeout_sec: Optional[float]=None) \
                      -> Optional[CollisionObjectInfo]:
        """ Create a new collision object at specified pose.

        Args:
          object_type: Type of the object to be created.
          pose:        Pose of `subframe` of the created object.
          subframe:    Subframe name with which the pose of the object
                       is specified.
          object_id:   Unique ID of the object to be created. Same string as
                       `object_type` will be assigned, if an empty string,
                       in default, is given.
          timeout_sec: Seconds to wait for the response.
                       If `None`, then wait forever.
        """
        req = ManageCollisionObject.Request()
        req.op          = ManageCollisionObject.Request.CREATE_OBJECT
        req.object_type = object_type
        req.object_id   = object_id if object_id != '' else object_type
        req.subframe    = subframe
        req.frame_id    = pose.header.frame_id
        req.pose        = pose.pose
        res = self._send(req, timeout_sec)
        return res.info if res and res.success else None

    def remove_object(self, object_id: str='', frame_id: str='',
                      *, timeout_sec: Optional[float]=None) \
                      -> Optional[CollisionObjectInfo]:
        """ Remove attached or non-attached collision object.

        Args:
          object_id:   Unique ID of the object to be removed. All non-attached
                       collision objects as well as collision_objects
                       attached to `frame_id` will be removed, if an empty
                       string, in default, is given.
          frame_id:    Frame ID to which attached collision objects
                       are attached to. All attached-collision object attached
                       to any frames will be removed, if an empty string,
                       in default, is given.
          timeout_sec: Seconds to wait for the response.
                       If `None`, then wait forever.
        """
        req = ManageCollisionObject.Request()
        req.op        = ManageCollisionObject.Request.REMOVE_OBJECT
        req.object_id = object_id
        req.frame_id  = frame_id
        res = self._send(req, timeout_sec)
        return res.info if res and res.success else None

    def attach_object(self, object_id: str, parent_link: str, leaf_id: str='',
                      *, timeout_sec: Optional[float]=None) \
                      -> Optional[CollisionObjectInfo]:
        req = ManageCollisionObject.Request()
        req.op        = ManageCollisionObject.Request.ATTACH_OBJECT
        req.object_id = object_id
        req.frame_id  = parent_link
        req.leaf_id   = leaf_id
        res = self._send(req, timeout_sec)
        return res.info if res and res.success else None

    def detach_object(self, object_id: str, parent_link: str, leaf_id: str='',
                      *, timeout_sec: Optional[float]=None):
        req = ManageCollisionObject.Request()
        req.op        = ManageCollisionObject.Request.DETACH_OBJECT
        req.object_id = object_id
        req.frame_id  = parent_link
        req.leaf_id   = leaf_id
        res = self._send(req, timeout_sec)
        return res.info if res and res.success else None

    def move_object(self, object_id: str, pose: PoseStamped,
                    subframe: str='base_link',
                    *, timeout_sec: Optional[float]=None) \
                    -> Optional[CollisionObjectInfo]:
        req = ManageCollisionObject.Request()
        req.op        = ManageCollisionObject.Request.MOVE_OBJECT
        req.object_id = object_id
        req.subframe  = subframe
        req.frame_id  = pose.header.frame_id
        req.pose      = pose.pose
        res = self._send(req, timeout_sec)
        return res.info if res and res.success else None

    def append_touch_links(self, object_id: str, frame_id: str,
                           *, timeout_sec: Optional[float]=None) \
                           -> Optional[CollisionObjectInfo]:
        req = ManageCollisionObject.Request()
        req.op        = ManageCollisionObject.Request.APPEND_TOUCH_LINKS
        req.object_id = object_id
        req.frame_id  = frame_id
        res = self._send(req, timeout_sec)
        return res.info if res and res.success else None

    def remove_touch_links(self, object_id: str, frame_id: str,
                           *, timeout_sec: Optional[float]=None) \
                           -> Optional[CollisionObjectInfo]:
        req = ManageCollisionObject.Request()
        req.op        = ManageCollisionObject.Request.REMOVE_TOUCH_LINKS
        req.object_id = object_id
        req.frame_id  = frame_id
        res = self._send(req, timeout_sec)
        return res.info if res and res.success else None

    def reset_touch_links(self, *, timeout_sec: Optional[float]=None)-> bool:
        req = ManageCollisionObject.Request()
        req.op = ManageCollisionObject.Request.RESET_TOUCH_LINKS
        return self._send(req, timeout_sec).success

    def get_object_info(self, object_id: str,
                        *, timeout_sec: Optional[float]=None) \
                        -> Optional[CollisionObjectInfo]:
        req           = ManageCollisionObject.Request()
        req.op        = ManageCollisionObject.Request.GET_OBJECT_INFO
        req.object_id = object_id
        res = self._send(req, timeout_sec)
        return res.info if res and res.success else None

    def get_child_object_info(self, frame_id: str,
                              *, timeout_sec: Optional[float]=None) \
                              -> Optional[CollisionObjectInfo]:
        req          = ManageCollisionObject.Request()
        req.op       = ManageCollisionObject.Request \
                      .GET_ATTACHED_CHILD_OBJECT_INFO
        req.frame_id = frame_id
        res = self._send(req, timeout_sec)
        return res.info if res and res.success else None

    def allow_collision(self, object_id, frame_id,
                        *, timeout_sec: Optional[float]=None)-> Optional[bool]:
        req           = ManageCollisionObject.Request()
        req.op        = ManageCollisionObject.Request.ALLOW_COLLISION
        req.object_id = object_id
        req.frame_id  = frame_id
        res = self._send(req, timeout_sec)
        return res.success if res else None

    def disallow_collision(self, object_id: str, frame_id: str,
                           *, timeout_sec: Optional[float]=None) \
                           -> Optional[bool]:
        req           = ManageCollisionObject.Request()
        req.op        = ManageCollisionObject.Request.DISALLOW_COLLISION
        req.object_id = object_id
        req.frame_id  = frame_id
        res = self._send(req, timeout_sec)
        return res.success if res else None

    def _send(self, req, timeout_sec):
        return self._client.call(req, timeout_sec=timeout_sec)
