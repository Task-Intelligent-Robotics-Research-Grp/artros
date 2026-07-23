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
import rclpy, yaml, threading
import numpy as np
import tf_transformations as tfs
import pyassimp

from rclpy.node                    import Node
from rclpy.callback_groups         import MutuallyExclusiveCallbackGroup
from rclpy.duration                import Duration
from rclpy.time                    import Time
from tf2_ros.transform_broadcaster import TransformBroadcaster
from rcl_interfaces.msg            import ParameterDescriptor, ParameterType
from std_msgs.msg                  import Header, ColorRGBA
from geometry_msgs.msg             import (Point, Vector3, Quaternion, Pose,
                                           Transform, TransformStamped,
                                           PoseStamped)
from shape_msgs.msg                import (Mesh, MeshTriangle, Plane,
                                           SolidPrimitive)
from visualization_msgs.msg        import Marker, MarkerArray
from aist_msgs.srv                 import GetCollisionObject
from aist_msgs.msg                 import (CollisionObjectInfo, LinkGeometry,
                                           Material)
from moveit_msgs.msg               import (CollisionObject,
                                           AttachedCollisionObject,
                                           PlanningSceneComponents,
                                           PlanningScene)
from moveit_msgs.srv               import GetPlanningScene
from moveit_commander              import planning_scene_interface as psi
from aist_utility.fileio           import filepath_from_url

from typing                        import List, Dict, Tuple, Optional

#************************************************************************
#  local functions                                                      *
#************************************************************************
def _decompose_link_name(link_name: str)-> Tuple[str, str]:
    """ Decompose the given link name into object_id and subframe.

    Args:
      link_name: name of the link

    Returns:
      A tuple of object ID and subframe name of the collision object,
      if `link_name` is a fullname of subframe of any collision object.
      A tuple ('', `link_name`), otherwise.

    Examples:
      * 'panel_bearing/base_link` => ('panel_bearing', 'base_link')
      * 'a_bot_gripper_tip_link'  => ('', 'a_bot_gripper_tip_link')
    """
    tokens = link_name.rsplit('/', 1)
    return tokens if len(tokens) == 2 else ('', link_name)

def _get_base_link(link_name: str)-> str:
    """ Convert the given link name to the fullname of the base subframe.

    Args:
      link_name: name of the link

    Returns:
      A tuple of object ID and subframe name of the collision object,
      if `link_name` is a fullname of subframe of any collision object.
      A tuple ('', `link_name`), otherwise.

    Examples:
      * 'panel_bearing/base_screw_hole_1` => 'panel_bearing/base_link'
      * 'a_bot_gripper_tip_link' => 'a_bot_gripper_tip_link'
    """
    object_id, _ = _decompose_link_name(link_name)
    return link_name if object_id == '' else object_id + '/base_link'

def _vector3_from_xyz(xyz: Tuple[float, float, float])-> Vector3:
    return Vector3(x=xyz[0], y=xyz[1], z=xyz[2])

def _color_from_rgba(rgba: Tuple[float, float, float, float])-> ColorRGBA:
    return ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])

def _pose_from_xyzrpy(xyzrpy):
    q = tfs.quaternion_from_euler(*np.radians(xyzrpy[3:6]))
    return Pose(position=Point(x=xyzrpy[0], y=xyzrpy[1], z=xyzrpy[2]),
                orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))

def _pose_matrix(pose):
    return tfs.translation_matrix((pose.position.x,
                                   pose.position.y,
                                   pose.position.z)) \
         @ tfs.quaternion_matrix((pose.orientation.x, pose.orientation.y,
                                  pose.orientation.z, pose.orientation.w))

def _pose_from_matrix(T):
    t = tfs.translation_from_matrix(T)
    q = tfs.quaternion_from_matrix(T)
    return Pose(position=Point(x=t[0], y=t[1], z=t[2]),
                orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))

def _transform_matrix(transform):
    return _pose_matrix(_pose_from_transform(transform))

def _transform_from_matrix(T):
    return _transform_from_pose(_pose_from_matrix(T))

def _pose_from_transform(transform: Transform)-> Pose:
    return Pose(position=Point(x=transform.translation.x,
                               y=transform.translation.y,
                               z=transform.translation.z),
                orientation=Quaternion(x=transform.rotation.x,
                                       y=transform.rotation.y,
                                       z=transform.rotation.z,
                                       w=transform.rotation.w))

def _transform_from_pose(pose: Pose)-> Transform:
    return Transform(translation=Vector3(x=pose.position.x,
                                         y=pose.position.y,
                                         z=pose.position.z),
                     rotation=Quaternion(x=pose.orientation.x,
                                         y=pose.orientation.y,
                                         z=pose.orientation.z,
                                         w=pose.orientation.w))

def _format_transform(transform: Transform)-> str:
    return '{} <= {}: [{:.4f}, {:.4f}, {:.4f}; {:.4f}, {:.4f}. {:.4f}. {:.4f}]' \
        .format(transform.header.frame_id, transform.child_frame_id,
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
                transform.transform.rotation.x, transform.transform.rotation.y,
                transform.transform.rotation.z, transform.transform.rotation.w)

def _format_pose(pose: Pose)-> str:
    return '[{:.4f}, {:.4f}, {:.4f}; {:.4f}, {:.4f}. {:.4f}. {:.4f}]' \
        .format(pose.position.x, pose.position.y, pose.position.z,
                pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w)

#************************************************************************
#  class CollisionObjectManager                                         *
#************************************************************************
class CollisionObjectManager(object):
    """ Python interface for managing collision objects.
    * Maintain tree structure of collision objects
    * Service server for responding to requests for mesh resource
    * Service server for responding to requests for managing collision objects
    * Publish subframes of collision objects to TF
    * Publish shape of collision objects to topic '~collision_marker'
      as visual markers
    """

    class ObjectProperties(object):
        def __init__(self):
            super().__init__()

            self.primitives            = []
            self.primitive_poses       = []
            self.visual_mesh_urls      = []
            self.visual_mesh_poses     = []
            self.visual_mesh_scales    = []
            self.visual_mesh_colors    = []
            self.collision_mesh_urls   = []
            self.collision_mesh_poses  = []
            self.collision_mesh_scales = []
            self.collision_meshes      = []
            self.subframe_names = ['base_link']
            self.subframe_poses = [Pose(position=Point(x=0.0, y=0.0, z=0.0),
                                        orientation=Quaternion(x=0.0, y=0.0,
                                                               z=0.0, w=1.0))]

    class InstanceProperties(object):
        def __init__(self, type):
            super().__init__()

            self.type                = type
            self.subframe_transforms = []
            self.markers             = []

        @property
        def base_link_transform(self):
            return self.subframe_transforms[0]

        @base_link_transform.setter
        def base_link_transform(self, base_link_transform):
            self.subframe_transforms[0] = base_link_transform

        @property
        def parent_link(self):
            return self.base_link_transform.header.frame_id

    def __init__(self, node: Node, ns: str=''):
        """ Create collision object manager.
        * Load object properties from parameter '~object_properties'
          for each type
        * Setup marker publisher '~collision_marker' as well as services
          'get_collision_object' and 'manage_collision_object'
        """
        super().__init__()

        def ns_join(ns, name):
            return '/'.join([ns, name]) if ns else name

        PRIMITIVES = {'BOX':      SolidPrimitive.BOX,
                      'SPHERE':   SolidPrimitive.SPHERE,
                      'CYLINDER': SolidPrimitive.CYLINDER,
                      'CONE':     SolidPrimitive.CONE}

        self._node = node

        # Create a dictionary of object properties loaded from database.
        self._obj_props_dict = {}
        for type, props in self._load_databases(
                               node.declare_parameter('object_properties_urls',
                                                      ['']).value).items():
            obj_props = CollisionObjectManager.ObjectProperties()

            for primitive in props.get('primitives', []):
                obj_props.primitives.append(
                    SolidPrimitive(type=PRIMITIVES[primitive['type']],
                                   dimensions=primitive['dimensions']))
                obj_props.primitive_poses.append(
                    _pose_from_xyzrpy(primitive['pose']))

            for subframe_name, subframe_pose in props.get('subframes',
                                                          {}).items():
                obj_props.subframe_names.append(subframe_name)
                obj_props.subframe_poses.append(
                    _pose_from_xyzrpy(subframe_pose))

            for mesh in props.get('visual_meshes', []):
                obj_props.visual_mesh_urls.append(mesh['url'])
                obj_props.visual_mesh_poses.append(
                    _pose_from_xyzrpy(mesh['pose']))
                obj_props.visual_mesh_scales.append(
                    _vector3_from_xyz(mesh['scale']))
                obj_props.visual_mesh_colors.append(
                    _color_from_rgba(mesh['color']))

            for mesh in props.get('collision_meshes', []):
                obj_props.collision_mesh_urls.append(mesh['url'])
                obj_props.collision_mesh_poses.append(
                    _pose_from_xyzrpy(mesh['pose']))
                obj_props.collision_mesh_scales.append(
                    _vector3_from_xyz(mesh['scale']))
                obj_props.collision_meshes.append(
                    self._load_mesh(mesh['url'], mesh['scale']))

            self._obj_props_dict[type] = obj_props
            self.logger.info('loaded properties of type[%s]' % type)

        # Create an instance of PlanningSceneInterface.
        self._psi = psi.PlanningSceneInterface(self, '',
                                               node.declare_parameter(
                                                   'synchronous', True).value)

        # Create a client of GetPlanningScene service.
        self._get_planning_scene \
            = node.create_client(
                GetPlanningScene, ns_join(ns, 'get_planning_scene'),
                callback_group=MutuallyExclusiveCallbackGroup())
        if not self._get_planning_scene.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('failed to establish connection to the service[get_planning_scene]')

        self._instance_props_dict = {}
        self._instance_props_lock = threading.Lock()
        self._touch_links         = self._load_databases(
                                        node.declare_parameter(
                                            'touch_links_urls', ['']).value)
        self._marker_id_min       = 0
        self._marker_id_lists     = {}
        self._marker_pub          = node.create_publisher(MarkerArray,
                                                          'collision_marker',
                                                          1)
        self._tf2_buffer          = node._tf2_buffer
        self._broadcaster         = TransformBroadcaster(node)
        self._timer               = node.create_timer(
                                        node.declare_parameter('period',
                                                               0.1).value,
                                        self._subframes_and_markers_cb,
                                        MutuallyExclusiveCallbackGroup())
        self._service_cbg         = MutuallyExclusiveCallbackGroup()
        self._get_collision_object \
            = node.create_service(GetCollisionObject, 'get_collision_object',
                                  self._get_collision_object_cb,
                                  callback_group=self._service_cbg)

    @property
    def node(self)-> Node:
        return self._node

    @property
    def logger(self):
        return self.node.get_logger()

    #
    # Operations
    #
    def create_object(self, object_type: str, pose: PoseStamped,
                      subframe: str='base_link', object_id: str='')-> bool:
        """ Create a new collision object.
        The created new collision object is not attached to any links
        and its pose is specified as that of subframe.

        Args:
          object_type: Type of the object to be created.
          pose:        Pose of `subframe` of the created object.
          subframe:    Subframe name with which the pose of the object
                       is specified.
          object_id:   ID of the object to be created. Same string as
                       `object_type` will be assigned, if an empty string
                       (default) is given.

        Returns:
          `True` on success, `False` on failure.
        """
        self.logger.info(
            "*CREATE_OBJECT*: object_type='%s', object_id='%s', pose=%s@'%s', subframe='%s'"
            % (object_type, object_id,
               self.node.format_pose(pose, pose.header.frame_id),
               pose.header.frame_id, subframe))

        obj_props = self._obj_props_dict.get(object_type)
        if obj_props is None:
            self.logger.error('unknown object type[%s]' % object_type)
            return False

        if object_id == '':
            object_id = object_type

        # Setup a new collision object.
        co = CollisionObject()
        co.id = object_id
        co.primitives      = obj_props.primitives
        co.primitive_poses = obj_props.primitive_poses
        co.meshes          = obj_props.collision_meshes
        co.mesh_poses      = obj_props.collision_mesh_poses
        co.subframe_names  = obj_props.subframe_names
        co.subframe_poses  = obj_props.subframe_poses
        co.operation       = CollisionObject.ADD

        # If the object pose is specified as that of subframe other than
        # 'base_link', convert the given pose to that of 'base_link'.
        # Then compute a transform from 'base_link' to the new parent link.
        pose = self._get_base_link_pose_from_subframe_pose(pose, co, subframe)
        co.header = pose.header
        co.pose   = pose.pose

        # Create a new collision object.
        self._psi.add_object(co)

        # Create info for this object.
        instance_props = CollisionObjectManager.InstanceProperties(object_type)

        # Create subframe transforms.
        base_link = object_id + '/base_link'
        instance_props.subframe_transforms \
            = [TransformStamped(header=Header(frame_id=pose.header.frame_id),
                                child_frame_id=base_link,
                                transform=_transform_from_pose(pose.pose))]
        for subframe_name, subframe_pose in zip(obj_props.subframe_names,
                                                obj_props.subframe_poses):
            if subframe_name != 'base_link':
                instance_props.subframe_transforms.append(
                    TransformStamped(
                        header=Header(frame_id=base_link),
                        child_frame_id=object_id + '/' + subframe_name,
                        transform=_transform_from_pose(subframe_pose)))

        # Create new marker IDs if not existing for this object.
        if object_id not in self._marker_id_lists:
            self._marker_id_lists[object_id] \
              = self._generate_marker_id_list(len(obj_props.visual_mesh_urls))

        # Create markers for visualization.
        for mesh_url, mesh_pose, mesh_scale, mesh_color, marker_id \
            in zip(obj_props.visual_mesh_urls,   obj_props.visual_mesh_poses,
                   obj_props.visual_mesh_scales, obj_props.visual_mesh_colors,
                   self._marker_id_lists[object_id]):
            marker = Marker()
            marker.header.frame_id = base_link
            marker.ns              = ''
            marker.id              = marker_id
            marker.type            = marker.MESH_RESOURCE
            marker.action          = Marker.ADD
            marker.pose            = mesh_pose
            marker.scale           = mesh_scale
            marker.color           = mesh_color
            marker.lifetime        = Duration(seconds=0).to_msg()
            marker.frame_locked    = False
            marker.mesh_resource   = mesh_url
            marker.text            = object_type
            instance_props.markers.append(marker)

        # Store object info.
        with self._instance_props_lock:
            self._instance_props_dict[object_id] = instance_props

        # Add the object to AllowedCollisionMatrix(acm) and disallow collision
        # against any other objects in default.
        self._set_acm_allowed(object_id, None, allow=False, reset=True)

        self.logger.info("created '%s' of type[%s]" % (co.id, object_type))
        return True

    def remove_object(self, object_id: str='', frame_id: str='')-> None:
        """ Remove attached or non-attached collision object.

        Args:
          object_id: ID of the object to be removed. If an empty
                     string(default) is given, all non-attached collision
                     objects as well as attached collision objects
                     attached to `frame_id` will be removed.
          frame_id:  ID of the frame to which attached collision objects
                     to be removed are attached. If an empty string(default)
                     is given, all attached collision objects will be removed.
        """
        self.logger.info("*REMOVE_OBJECT*: object_id='%s', frame_id='%s'"
                         % (object_id, frame_id))

        if object_id != '':
            self._delete_markers_and_subframes(object_id)
        elif frame_id != '':
            object_id = None
            for aco in self._psi.get_attached_objects().values():
                if aco.link_name == frame_id:
                    self._delete_markers_and_subframes(aco.object.id)
        else:
            object_id = None
            frame_id = None
            for co_id in self._psi.get_objects().keys():
                self._delete_markers_and_subframes(co_id)
            for aco_id in self._psi.get_attached_objects().keys():
                self._delete_markers_and_subframes(aco_id)
        self._psi.remove_attached_object(frame_id, object_id)
        self._psi.remove_world_object(object_id)

    def attach_object(self, object_id: str, parent_link: str,
                      leaf_id: str='')-> bool:
        """ Attach collision object to the specified link.

        Args:
          object_id:   ID of the first object to be attached.
          parent_link: ID of link with which 'object_id' will be made contact.
          leaf_id:     ID of the past-of-last object to be attached.

        Returns:
          `True` on success, `False` on failure.
        """
        self.logger.info(
            "*ATTACH_OBJECT*: object_id='%s', parent_link='%s', leaf_id='%s'"
            % (object_id, parent_link, leaf_id))

        co = self._find_object(object_id)
        if co is None:
            self.logger.error("unknown object '%s'" % object_id)
            return False

        # Make this object root of the tree to be attached.
        old_root_id, old_parent_link = self._rotate_tree(co, leaf_id)

        # If 'parent_link' is a subframe of another collision object,
        # get fullname of its 'base_link'.
        parent_link = _get_base_link(parent_link)

        # Lookup transform from 'base_link' of the current collision object
        # to the parent link. Don't lookup within the block locking
        # _instance_props_dict because _subframes_and_markers_cb would be
        # blocked and prevents looking-up subframes.
        Tpo = self._tf2_buffer.lookup_transform(parent_link,
                                                co.id + '/base_link',
                                                self.node.get_clock().now(),
                                                Duration(seconds=2))

        # Set the parent of 'co' to 'parent_link'.
        with self._instance_props_lock:
            self._instance_props_dict[co.id].base_link_transform = Tpo

        # Get 'attach_link', i.e. a link to which 'co' will be attached.
        # - It should be 'parent_link', if it is not a 'base_link'
        #   of another object.
        # - It should be touch link of the parent object, otherwise.
        # A transform from 'co' to 'attach_link' is also computed.
        parent_co = self._find_object(_decompose_link_name(parent_link)[0])
        if parent_co is None:
            attach_link = parent_link
            Tao = _transform_matrix(Tpo.transform)
        else:
            attach_link = parent_co.header.frame_id  # attach link of parent
            Tao = _pose_matrix(parent_co.pose) \
                @ _transform_matrix(Tpo.transform)

        # Attach 'co' and its descendants to 'attach_link' with 'pose'
        # described w.r.t. 'attach_link'.
        self._attach_descendants(co, attach_link,
                                 Tao @
                                 tfs.inverse_matrix(_pose_matrix(co.pose)))
        self.allow_collision(old_root_id, old_parent_link)
        return True

    def detach_object(self, object_id: str, parent_link: str,
                      leaf_id: str)-> bool:
        """ Dettach collision object and make it contact with the specified
            link.

        Args:
          object_id:   ID of the first object to be dettached.
          parent_link: Name of link with which the object will be made contact.
          leaf_id:     ID of the past-of-last object to be dettached.

        Returns:
          `True` on success, `False` on failure.
        """
        self.logger.info(
            "*DETACH_OBJECT*: object_id='%s', parent_link='%s', leaf_id='%s'"
            % (object_id, parent_link, leaf_id))

        aco = self._find_attached_collision_object(object_id)
        if aco is None:
            self.logger.error("unknown attached collision object '%s'"
                              % object_id)
            return False

        # Make this object root of the tree to be dettached.
        old_root_id, old_parent_link = self._rotate_tree(aco.object, leaf_id)

        # If 'parent_link' is a subframe of another collision object,
        # get fullname of its 'base_link'.
        parent_link = _get_base_link(parent_link)

        # Lookup transform from 'base_link' of the current collision object
        # to the parent link. Don't lookup within the block locking
        # _instance_props_dict because _subframes_and_markers_cb would be
        # blocked and prevents looking-up subframes.
        Tpo = self._tf2_buffer.lookup_transform(parent_link,
                                                aco.object.id + '/base_link',
                                                self.node.get_clock().now(),
                                                Duration(seconds=2))
        with self._instance_props_lock:
            self._instance_props_dict[aco.object.id].base_link_transform = Tpo

        # Detach 'aco' from its attach link.
        self.logger.info("detached '%s' from '%s'"
                         % (aco.object.id, aco.link_name))
        co = aco.object
        self.logger.warn('### Before: co.header.frame_id=%s'
                         % co.header.frame_id)
        self._psi.remove_attached_object(name=aco.object.id)
        co = self._find_collision_object(aco.object.id)
        self.logger.warn('### After: co.header.frame_id=%s'
                         % co.header.frame_id)

        # Since all child attached objects have contacts with the current
        # object 'co', we have to switch their attach links to 'link'.

        # Tao = _pose_matrix(co.pose)
        # attach_link =
        co = self._find_collision_object(aco.object.id)
        for child_aco in self._psi.get_attached_objects().values():
            if self._get_parent_id(child_aco.object.id) == co.id:
                self._attach_descendants(child_aco.object, co.header.frame_id,
                                         _pose.matrix(co.pose) @
                                         tfs.inverse_matrix(
                                             _pose_matrix(aco.object.pose)))
        #self._append_or_remove_touch_links(old_root_id, old_parent_link, True)
        return True

    def move_object(self, object_id: str, pose: PoseStamped,
                    subframe: str='base_link')-> bool:
        self.logger.info(
            "*MOVE_OBJECT*: object_id='%s', pose=%s@'%s', subframe='%s'"
            % (object_id, self.node.format_pose(pose, pose.header.frame_id),
               pose.header.frame_id, subframe))

        co = self._find_object(object_id)
        if co is None:
            self.logger.error("unknown collision object '%s'" % object_id)
            return False

        # Transform the given pose from 'frame_id' to parent link of 'co'.
        parent_link = self._get_parent_link(co.id)
        now = self.node.get_clock().now()
        pose.pose = _pose_from_matrix(
                         _transform_matrix(
                             self._tf2_buffer.lookup_transform(
                                 parent_link, pose.header.frame_id,
                                 now, Duration(seconds=2)).transform) @
                         _pose_matrix(pose.pose))
        pose.header.frame_id = parent_link

        # Transform the given pose of subframe to that of 'base_link'
        # described w.r.t. 'parent_link' which is a parent link of 'object_id'.
        pose = self._get_base_link_pose_from_subframe_pose(pose, co, subframe)
        with self._instance_props_lock:
            self._instance_props_dict[co.id].base_link_transform \
                = TransformStamped(header=Header(frame_id=parent_link),
                                   child_frame_id=co.id + '/base_link',
                                   transform=_transform_from_pose(pose.pose))
        self._move_descendants(co,
                               _transform_matrix(
                                   self._tf2_buffer.lookup_transform(
                                       co.header.frame_id, parent_link, now,
                                       Duration(seconds=2)).transform) @ \
                               tfs.inverse_matrix(_pose_matrix(co.pose)))
        return True

    def allow_collision(self, object_id: str, frame_id: str)-> None:
        self.logger.info("*ALLOW_COLLISION*: object_id='%s', frame_id='%s'"
                         % (object_id, frame_id))

        touch_links = self._get_touch_links(frame_id)
        self._set_acm_allowed(object_id, touch_links, allow=True, reset=False)

        aco = self._find_attached_collision_object(object_id)
        if aco is not None:
            touch_links = list(set(aco.touch_links) | set(touch_links))
            self._psi.attach_object(aco, touch_links=touch_links)

    def disallow_collision(self, object_id: str, frame_id: str)-> None:
        self.logger.info("*%DISALLOW_COLLISION*: object_id='%s', frame_id='%s'"
                         % (object_id, frame_id))

        touch_links = self._get_touch_links(frame_id)
        self._set_acm_allowed(object_id, touch_links, allow=False, reset=False)

        aco = self._find_attached_collision_object(object_id)
        if aco is not None:
            touch_links = list(set(aco.touch_links) - set(touch_links))
            self._psi.attach_object(aco, touch_links=touch_links)

    def reset_collision(self, object_id: str)-> None:
        """ Reset ACM entries and touch links of the specified object.
        Update ACM entries of the specified object so that collision
        against only touch links associated with the parent link of it
        is allowed. If the object is an attached collision object,
        its touch links are also updated.

        Args:
          object_id: ID of the object whose ACM and touch links to be reset.
        """
        self.logger.info("*RESET_COLLISION*: object_id='%s'" % object_id)

        touch_links = self._get_parent_touch_links(object_id)
        self._set_acm_allowed(object_id, touch_links, allow=True, reset=True)

        aco = self._find_attached_collision_object(object_id)
        if aco is not None:
            self._psi.attach_object(aco, touch_links=touch_links)

    def get_object_info(self, object_id: str)-> Optional[CollisionObjectInfo]:
        """ Get information on attached or non-attached collision object.

        Args:
          object_id: ID of the object.

        Returns:
          Information on the object specified by `object_id`.
        """
        self.logger.info("*GET_OBJECT_INFO*: object_id='%s'" % object_id)

        info = CollisionObjectInfo()
        info.object_id = object_id
        co = self._find_collision_object(object_id)
        if co is None:
            aco = self._find_attached_collision_object(object_id)
            if aco is None:
                self.logger.error("unknown object '%s'" % object_id)
                return None
            info.attach_link = aco.link_name
            info.touch_links = aco.touch_links
            info.pose        = PoseStamped(header=aco.object.header,
                                           pose=aco.object.pose)
        else:
            info.pose = PoseStamped(header=co.header, pose=co.pose)
        info.object_type = self._instance_props_dict[object_id].type
        info.parent_link = self._get_parent_link(object_id)
        info.acm_allowed = self._get_acm_allowed_entries(object_id)
        return info

    def get_attached_child_object_info(self, frame_id: str) \
            -> Optional[CollisionObjectInfo]:
        self.logger.info("*GET_ATTACHED_CHILD_OBJECT_INFO*: frame_id='%s'"
                         % frame_id)

        for aco in self._psi.get_attached_objects().values():
            if self._get_parent_link(aco.object.id) == frame_id:
                info = CollisionObjectInfo()
                info.object_id   = aco.object.id
                info.attach_link = aco.link_name
                info.touch_links = aco.touch_links
                info.pose        = PoseStamped(header=aco.object.header,
                                               pose=aco.object.pose)
                info.object_type = self._instance_props_dict[info.object_id] \
                                       .type
                info.parent_link = self._get_parent_link(info.object_id)
                return info

        self.logger.error("no attached collision objects connected to '%s' found"
                          % frame_id)
        return None

    #
    # Callbacks
    #
    def _subframes_and_markers_cb(self):
        """ Timer callback.
        Publish subframes and visual markers periodically.
        """
        now = self.node.get_clock().now().to_msg()
        transforms = []
        markers = []
        with self._instance_props_lock:
            for instance_props in self._instance_props_dict.values():
                for subframe_transform in instance_props.subframe_transforms:
                    subframe_transform.header.stamp = now
                    transforms.append(subframe_transform)
                for marker in instance_props.markers:
                    marker.header.stamp = now
                    markers.append(marker)
        self._broadcaster.sendTransform(transforms)
        self._marker_pub.publish(MarkerArray(markers=markers))

    def _get_collision_object_cb(self, req, res):
        """ Service callback for GetCollisionObject.
        Send response with binary mesh data according to the requested URL
        of mesh resource.
        """
        self.logger.info('GetCollisionObject[object_type=%s]'
                         % req.object_type)

        obj_props = self._obj_props_dict.get(req.object_type)
        if not obj_props:
            self.logger.error('Unknown obejct type[%s]' % req.object_type)
            return

        try:
            res.visual_array = [self._create_link_mesh(mesh_url,
                                                       mesh_pose, mesh_scale)
                                for mesh_url, mesh_pose, mesh_scale
                                in zip(obj_props.visual_mesh_urls,
                                       obj_props.visual_mesh_poses,
                                       obj_props.visual_mesh_scales)]
            if not obj_props.primitives:
                res.collision_array = [self._create_link_mesh(mesh_url,
                                                              mesh_pose,
                                                              mesh_scale)
                                       for mesh_url, mesh_pose, mesh_scale
                                       in zip(obj_props.collision_mesh_urls,
                                              obj_props.collision_mesh_poses,
                                              obj_props.collision_mesh_scales)]
            else:
                res.collision_array = [self._create_link_primitive(
                                           primitive, primitive_pose)
                                       for primitive, primitive_pose
                                       in zip(obj_props.primitives,
                                              obj_props.primitive_poses)]
            res.material_array = [self._create_link_material(mesh_color)
                                  for mesh_color
                                  in obj_props.visual_mesh_colors]
        except Exception as e:
            self.logger.error('_get_collision_object_cb(): %s' % e)

        return res

    #
    # Utilities
    #
    @staticmethod
    def _load_databases(urls: List[str])-> Dict:
        databases = {}
        for url in urls:
            with open(filepath_from_url(url), 'r') as f:
                databases |= yaml.safe_load(f)
        return databases

    @staticmethod
    def _load_mesh(url: str,
                   scale: Tuple[float, float, float]=(0.001, 0.001, 0.001)) \
                   -> Mesh:
        with pyassimp.load(filepath_from_url(url)) as scene:
            if not scene.meshes or len(scene.meshes) == 0:
                raise RuntimeError("no meshes in the file")
            if len(scene.meshes[0].faces) == 0:
                raise RuntimeError("no faces in the mesh")

        mesh = Mesh()
        first_face = scene.meshes[0].faces[0]
        if hasattr(first_face, '__len__'):
            for face in scene.meshes[0].faces:
                if len(face) == 3:
                    triangle = MeshTriangle()
                    triangle.vertex_indices = [face[0], face[1], face[2]]
                    mesh.triangles.append(triangle)
        elif hasattr(first_face, 'indices'):
            for face in scene.meshes[0].faces:
                if len(face.indices) == 3:
                    triangle = MeshTriangle()
                    triangle.vertex_indices = [face.indices[0],
                                               face.indices[1],
                                               face.indices[2]]
                    mesh.triangles.append(triangle)
        else:
            raise RuntimeError("unable to build triangles from mesh due to mesh object structure")
        for vertex in scene.meshes[0].vertices:
            mesh.vertices.append(Point(x=vertex[0]*scale[0],
                                       y=vertex[1]*scale[1],
                                       z=vertex[2]*scale[2]))
        return mesh

    @staticmethod
    def _create_link_mesh(mesh_url, mesh_pose, mesh_scale):
        link_geometry = LinkGeometry()
        link_geometry.origin = mesh_pose
        link_geometry.primitive.type = 0  # Mesh
        link_geometry.primitive.dimensions = [mesh_scale.x, mesh_scale.y,
                                              mesh_scale.z]
        with open(filepath_from_url(mesh_url), 'rb') as f:
            link_geometry.data = f.read()
        return link_geometry

    @staticmethod
    def _create_link_primitive(primitive, primitive_pose):
        return LinkGeometry(origin=primitive_pose, primitive=primitive)

    @staticmethod
    def _create_link_material(color):
        return Material(color=color, texture_height=0, texture_width=0)

    def _rotate_tree(self,
                     co: CollisionObject, leaf_id: str)-> Tuple[str, str]:
        """ Rotate transformation tree so that the given collision object
        become root.
        Starting at the given collision object, this functions ascends the
        transformation tree until one of the following three conditions is
        met:
        * Reached a non-attached collision object.
        * Reahced an attached collision object whose parent is not an object
          of any type.
        * Reached an attached collision object whose parent is an object
          with the specified object ID.

        Args:
          co:      Attached or non-attached collision object to be root.
          leaf_id:
        """
        def _inverse_transform(transform: TransformStamped)-> TransformStamped:
            return TransformStamped(
                       header=Header(frame_id=transform.child_frame_id),
                       child_frame_id=transform.header.frame_id,
                       transform=_transform_from_matrix(
                           tfs.inverse_matrix(
                               _transform_matrix(transform.transform))))

        # If 'co' is non-attached collision object, we have reached root!
        if self._find_attached_collision_object(co.id) is None:
            self._psi.attach_object(co, co.header.frame_id)
            return co.id, self._get_parent_link(co.id)

        # If parent of 'co' is not a any type of collision object
        # or is an object with 'leaf_id', we have reached root!
        parent_co = self._find_object(self._get_parent_id(co.id))
        if parent_co is None or parent_co.id == leaf_id:
            return co.id, self._get_parent_link(co.id)

        # Ascend transformation tree until reached the root.
        old_root_id, old_parent_link = self._rotate_tree(parent_co, leaf_id)

        # Reverse parent-child relation between 'co' and 'parent_co'
        # in the transformation tree. However, touch links are kept unchanged.
        with self._instance_props_lock:
            self._instance_props_dict[parent_co.id].base_link_transform \
                = _inverse_transform(
                      self._instance_props_dict[co.id].base_link_transform)
        return old_root_id, old_parent_link

    def _attach_descendants(self, co: CollisionObject, attach_link: str,
                            Tao)-> None:
        """ Attach the collision object and its descendants to specified link.

        Args:
          co:          Collision object.
          attach_link: Name of link to which `co` and its descendants attached.
          Tao:         Traansform matrix from `co` to `attach_link`
        """
        # Attach 'co' to 'attach_link'.
        co.header.frame_id = attach_link
        co.pose = _pose_from_matrix(Tao @ _pose_matrix(co.pose))
        touch_links = self._get_parent_touch_links(co.id)
        self._psi.attach_object(co, attach_link, touch_links)
        self.logger.info("attached '%s' to '%s'@%s with touch_links%s"
                         % (co.id, attach_link, _format_pose(co.pose),
                            touch_links))

        # Since all child attached objects are connected to the current
        # object 'co', we have to switch their attach links to 'attach_link'.
        for child_aco in self._psi.get_attached_objects().values():
            if self._get_parent_id(child_aco.object.id) == co.id:
                self._attach_descendants(child_aco.object, attach_link, Tao)

    def _move_descendants(self, co: CollisionObject, T)-> None:
        co.pose = _pose_from_matrix(T @ _pose_matrix(co.pose))
        aco = self._find_attached_collision_object(co.id)
        if aco is None:
            self._psi.add_object(co)
        else:
            self._psi.attach_object(co, aco.link_name, aco.touch_links)

        # Set poses for all child attached objects.
        for child_aco in self._psi.get_attached_objects().values():
            if self._get_parent_id(child_aco.object.id) == co.id:
                self._move_descendants(child_aco.object, T)

    def _get_base_link_pose_from_subframe_pose(self, pose: PoseStamped,
                                               co: CollisionObject,
                                               subframe: str)-> PoseStamped:
        """ Convert subframe pose of collision object to base link pose.

        Args:
          pose:     Pose of `subframe`.
          co:       Collision object.
          subframe: Subframe name with which the pose of `co` is specified.

        Returns:
          Pose of 'base_link' of `co`. If 'pose.header.frame_id' is a subframe
          of any other collision object, the pose is described w.r.t.
          'base_link' of that object.
        """
        def _subframe_pose(co, subframe):
            return co.subframe_poses[co.subframe_names.index(subframe)]

        # Convert the given pose of 'subframe' of 'co' to that of 'base_link'.
        pose.pose = _pose_from_matrix(_pose_matrix(pose.pose) @
                                      tfs.inverse_matrix(
                                          _pose_matrix(
                                              _subframe_pose(co, subframe))))

        # Separate the parent link 'pose.header.frame_id' into object ID
        # and subframe name.
        parent_id, parent_subframe = _decompose_link_name(pose.header.frame_id)

        # If the parent link is a subframe of any other collision object,
        # return its 'base_link' and the pose of 'base_link' of 'co'
        # w.r.t. it.
        if parent_id != '':
            pose.header.frame_id = parent_id + '/base_link'
            pose.pose = _pose_from_matrix(
                            _pose_matrix(
                                _subframe_pose(self._find_object(parent_id),
                                               parent_subframe)) @
                            _pose_matrix(pose.pose))
        return pose

    def _find_collision_object(self, object_id: str) \
            -> Optional[CollisionObject]:
        """ Find non-attached collision object with specified object ID.

        Args:
          object_id: ID of the object to be searched for.

        Returns:
          * Collision object with `object_id`, if found.
          * `None`, if not found.
        """
        return self._psi.get_objects([object_id]).get(object_id)

    def _find_attached_collision_object(self, object_id: str) \
            ->Optional[AttachedCollisionObject]:
        """ Find attached collision object with specified object ID.

        Args:
          object_id: ID of the object to be searched for.

        Returns:
          * Attached collision object with `object_id`, if found.
          * `None`, if not found.
        """
        return self._psi.get_attached_objects([object_id]).get(object_id)

    def _find_object(self, object_id: str)-> Optional[CollisionObject]:
        """ Find attached or non-attached collision object
        with specified object ID.

        Args:
          object_id: ID of the object to be searched for.

        Returns:
          * Collision object part of the attached collision object or
            collision object with `object_id`, if found.
          * `None`, if neighter found.
        """
        aco = self._find_attached_collision_object(object_id)
        return self._find_collision_object(object_id) if aco is None else \
               aco.object

    def _get_parent_link(self, object_id: str)-> str:
        """ Get parent link of the attached or non-attached collision object.

        Args:
          object_id: ID of the object to be searched for.

        Returns:
          Parent link of the object specified by `object_id`.
        """
        return self._instance_props_dict[object_id].parent_link

    def _get_parent_id(self, object_id: str)-> str:
        """ Get object ID of the parent of attached or non-attached collision
        object with specified object ID.

        Args:
          object_id: ID of the object to be searched for.

        Returns:
          * ID of the parent of `object_id`, if it is an attached or
            non-attached collision object.
          * An empty string, if not.
        """
        return _decompose_link_name(self._get_parent_link(object_id))[0]

    def _get_touch_links(self, link: str)-> List[str]:
        """ Get touch links associated with the specified link.

        Args:
          link: Name of the link whose touch links are searched for.

        Returns:
          * A list with only one element, unique ID of the object, if `link`
            represents a subframe of either an attached or a non-attached
            collision object.
          * Touch links associated with `link`, if `link` represents neither
            an attached nor a non-attached collision object.
        """
        object_id, _ = _decompose_link_name(link)
        return self._touch_links.get(link, []) if object_id == '' else \
               [object_id]

    def _get_parent_touch_links(self, object_id)-> List[str]:
        return self._get_touch_links(self._get_parent_link(object_id))

    def _generate_marker_id_list(self, n: int)-> List[int]:
        marker_id_list = []
        for i in range(n):
            marker_id_list.append(self._marker_id_min)
            self._marker_id_min += 1
        return marker_id_list

    def _delete_markers_and_subframes(self, object_id: str)-> None:
        instance_props = self._instance_props_dict.get(object_id)
        if instance_props is None:
            self.logger.error('unknown object[%s]' % object_id)
            return
        self._marker_pub.publish(
            MarkerArray(markers=[Marker(id=marker.id, action=Marker.DELETE)
                                 for marker in instance_props.markers]))
        with self._instance_props_lock:
            del self._instance_props_dict[object_id]
        self.logger.info("removed '%s'" % object_id)

    def _set_acm_allowed(self, object_id: str, other_links: List[str],
                         *, allow: bool, reset: bool)-> None:
        # Create a new ACM by modifying the existing one.
        acm = self._get_planning_scene.call(
                  GetPlanningScene.Request(component=PlanningSceneComponents \
                                           .ALLOWED_COLLISION_MATRIX)) \
                  .scene.allowed_collision_matrix
        if reset:
            for other_link in acm.entry_names:
                acm.set_allowed(object_id, other_link, False)

        if other_links is None:
            acm.set_default(object_id, allow)
            self.logger.info("%s '%s' collision by default"
                             % ('allow' if allow else 'disallow', object_id))
        else:
            for other_link in other_links:
                acm.set_allowed(object_id, other_link, allow)
            self.logger.info("%s '%s' collision against %s"
                             % ('allow' if allow else 'disallow',
                                object_id, other_links))

        # Apply the created ACM to the planning scene.
        scene = PlanningScene()
        scene.allowed_collision_matrix = acm
        scene.is_diff = True
        scene.robot_state.is_diff = True
        self._psi.apply_planning_scene(scene)

    def _get_acm_allowed_entries(self, object_id: str)-> List[str]:
        acm = self._get_planning_scene.call(
                  GetPlanningScene.Request(component=PlanningSceneComponents \
                                           .ALLOWED_COLLISION_MATRIX)) \
                  .scene.allowed_collision_matrix
        if object_id in acm.entry_names:
            entry_values = acm.entry_values[acm.entry_names.index(object_id)]
            return [name for j, name in enumerate(acm.entry_names)
                    if entry_values.enabled[j]]
        elif object_id in acm.default_entry_names and \
             acm.default_entry_values[acm.default_entry_names \
                                      .index(object_id)]:
            return ['ANY']
        else:
            return []
