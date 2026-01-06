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
import os, sys, yaml, copy, rclpy, threading
import numpy as np
import tf_transformations as tfs
import pyassimp

from collections                   import namedtuple
from rclpy.node                    import Node
from rclpy.executors               import MultiThreadedExecutor
from rclpy.callback_groups         import MutuallyExclusiveCallbackGroup
from rclpy.duration                import Duration
from rclpy.time                    import Time
from tf2_ros.buffer                import Buffer
from tf2_ros.transform_listener    import TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster
from rcl_interfaces.msg            import ParameterDescriptor, ParameterType
from std_msgs.msg                  import Header, ColorRGBA
from geometry_msgs.msg             import (Point, Vector3, Quaternion, Pose,
                                           Transform, TransformStamped,
                                           PoseStamped)
from shape_msgs.msg                import (Mesh, MeshTriangle, Plane,
                                           SolidPrimitive)
from visualization_msgs.msg        import Marker, MarkerArray
from aist_msgs.srv                 import (ManageCollisionObject,
                                           GetCollisionObject)
from aist_msgs.msg                 import CollisionObjectInfo
from moveit_msgs.msg               import (CollisionObject,
                                           AttachedCollisionObject,
                                           PlanningSceneComponents,
                                           PlanningScene)
from moveit_msgs.srv               import GetPlanningScene
from moveit_commander              import planning_scene_interface as psi
from aist_utility.fileio           import filepath_from_url

#########################################################################
#  local functions                                                      #
#########################################################################
def _decompose_link_name(link_name):
    tokens = link_name.rsplit('/', 1)
    return tokens if len(tokens) == 2 else ('', link_name)

def _get_base_link(frame_id):
    parent_id, _ = _decompose_link_name(frame_id)
    return frame_id if parent_id == '' else parent_id + '/base_link'

def _vector3_from_xyz(xyz):
    return Vector3(x=xyz[0], y=xyz[1], z=xyz[2])

def _color_from_rgba(rgba):
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

def _pose_from_transform(transform):
    return Pose(position=Point(x=transform.translation.x,
                               y=transform.translation.y,
                               z=transform.translation.z),
                orientation=Quaternion(x=transform.rotation.x,
                                       y=transform.rotation.y,
                                       z=transform.rotation.z,
                                       w=transform.rotation.w))

def _transform_from_pose(pose):
    return Transform(translation=Vector3(x=pose.position.x,
                                         y=pose.position.y,
                                         z=pose.position.z),
                     rotation=Quaternion(x=pose.orientation.x,
                                         y=pose.orientation.y,
                                         z=pose.orientation.z,
                                         w=pose.orientation.w))

#########################################################################
#  class CollisionObjectManager                                         #
#########################################################################
class CollisionObjectManager(Node):
    """Python interface for managing collision objects

    - Maintain tree structure of collision objects
    - Service server for responding to requests for mesh resource
    - Service server for responding to requests for managing collision objects
    - Publish subframes of collision objects to TF
    - Publish shape of collision objects to topic '~collision_marker'
      as visual markers
    """

    ObjectProperties = namedtuple('ObjectProperties',
                                  ['primitives', 'primitive_poses',
                                   'visual_mesh_urls', 'visual_mesh_poses',
                                   'visual_mesh_scales', 'visual_mesh_colors',
                                   'collision_mesh_urls',
                                   'collision_mesh_poses',
                                   'collision_mesh_scales', 'collision_meshes',
                                   'subframe_names', 'subframe_poses'])
    class InstanceProperties(object):
        def __init__(self, type):
            self.type                = type
            self.subframe_transforms = []
            self.markers             = []

        @property
        def parent_link(self):
            return self.subframe_transforms[0].header.frame_id

    def __init__(self, name):
        """Initialize collision object manager

        - Load object properties from parameter '~object_properties'
          for each type
        - Setup marker publisher '~collision_marker' as well as services
          '~get_collision_object' and '~manage_collision_object'
        """
        super().__init__(name)

        def ns_join(ns, name):
            return '/'.join([ns, name]) if ns else name

        PRIMITIVES   = {'BOX':      SolidPrimitive.BOX,
                        'SPHERE':   SolidPrimitive.SPHERE,
                        'CYLINDER': SolidPrimitive.CYLINDER,
                        'CONE':     SolidPrimitive.CONE}
        STR_ARY_DESC = ParameterDescriptor(
                           type=ParameterType.PARAMETER_STRING_ARRAY)
        self._psi = None

        # Create a dictionary of object properties loaded from database.
        self._obj_props_dict = {}
        for type, props in self._load_databases(
                               self.declare_parameter('object_properties_urls',
                                                      ['']).value).items():
            obj_props = CollisionObjectManager.ObjectProperties(
                            [], [],          # collision primitives
                            [], [], [], [],  # visual mesh properties
                            [], [], [], [],  # collision mesh properties
                            ['base_link'],   # subframe names
                            [Pose(position=Point(x=0.0, y=0.0, z=0.0),
                                  orientation=Quaternion(x=0.0, y=0.0,
                                                         z=0.0, w=1.0))])
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
            self.get_logger().info('loaded properties of type[%s]' % type)

        ns = self.declare_parameter('namespace', '').value

        # Create an instance of PlanningSceneInterface.
        self._psi = psi.PlanningSceneInterface(self, ns,
                                               self.declare_parameter(
                                                   'synchronous', True).value)

        # Create a client of GetPlanningScene service.
        self._get_planning_scene_cbg = MutuallyExclusiveCallbackGroup()
        self._get_planning_scene \
            = self.create_client(GetPlanningScene,
                                 ns_join(ns, 'get_planning_scene'),
                                 callback_group=self._get_planning_scene_cbg)
        if not self._get_planning_scene.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('failed to establish connection to the service[get_planning_scene]')

        self._instance_props_dict   = {}
        self._touch_links           = self._load_databases(
                                          self.declare_parameter(
                                              'touch_links_urls', ['']).value)
        self._marker_id_min         = 0
        self._marker_id_lists       = {}
        self._marker_pub            = self.create_publisher(
                                          MarkerArray, '~/collision_marker',
                                          10)
        self._tf2_buffer            = Buffer()
        self._tf2_listener          = TransformListener(self._tf2_buffer, self)
        self._broadcaster           = TransformBroadcaster(self)
        self._lock                  = threading.Lock()
        self._timer_cbg             = MutuallyExclusiveCallbackGroup()
        self._timer                 = self.create_timer(
                                          0.1, self._subframes_and_markers_cb,
                                          self._timer_cbg)
        self._get_collision_object \
            = self.create_service(GetCollisionObject, '~/get_collision_object',
                                  self._get_collision_object_cb)
        self._manage_collision_object_cbg = MutuallyExclusiveCallbackGroup()
        self._manage_collision_object \
            = self.create_service(
                  ManageCollisionObject, '~/manage_collision_object',
                  self._manage_collision_object_cb,
                  callback_group=self._manage_collision_object_cbg)

    #
    # File loaders
    #
    def _load_databases(self, urls):
        databases = {}
        for url in urls:
            with open(filepath_from_url(url), 'r') as f:
                databases |= yaml.safe_load(f)
        return databases

    def _load_mesh(self, url, scale=(0.001, 0.001, 0.001)):
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

    #
    # Callbacks
    #
    def _subframes_and_markers_cb(self):
        """Timer callback

        Publish subframes and visual markers periodically
        """
        with self._lock:
            for instance_props in self._instance_props_dict.values():
                for subframe_transform in instance_props.subframe_transforms:
                    subframe_transform.header.stamp \
                        = self.get_clock().now().to_msg()
                    self._broadcaster.sendTransform(subframe_transform)
                self._marker_pub.publish(
                    MarkerArray(markers=instance_props.markers))

    def _get_collision_object_cb(self, req, res):
        """Service callback for GetCollisionObject

        Send response with binary mesh data according to the requested URL
        of mesh resource
        """
        obj_props = self._obj_props_dict.get(req.object_type)
        if not obj_props:
            self.get_logger().error('Unknown obejct type[%s]'
                                    % req.object_type)
            return

        res.visual_array = [self._create_link_geometry(mesh_url, mesh_pose,
                                                       mesh_scale)
                            for mesh_url, mesh_pose, mesh_scale \
                                in zip(obj_props.visual_mesh_urls,
                                       obj_props.visual_mesh_poses,
                                       obj_props.visual_mesh_scales)]
        if not obj_props.primitives:
            res.collision_array = [self._create_link_geometry(mesh_url,
                                                              mesh_pose,
                                                              mesh_scale)
                                   for mesh_url, mesh_pose, mesh_scale \
                                       in zip(obj_props.collision_mesh_urls,
                                              obj_props.collision_mesh_poses,
                                              obj_props.collision_mesh_scales)]
        else:
            res.collision_array = [self._create_link_primitive(primitive,
                                                               primitive_pose)
                                   for primitive, primitive_pose \
                                       in zip(obj_props.primitives,
                                              obj_props.primitive_poses)]
        res.material_array = [self._create_material(mesh_color)
                              for mesh_color in obj_props.visual_mesh_colors]


        for obj_props in self._obj_props_dict.values():
            if req.mesh_resource in obj_props.visual_mesh_urls or \
               req.mesh_resource in obj_props.collision_mesh_urls:
                with open(filepath_from_url(req.mesh_resource), 'rb') as f:
                    res.data = f.read()
                self.get_logger().info('Send response to GetCollisionObject request for the mesh_url[%s]' % req.mesh_resource)
                break
        else:
            self.get_logger().error('Received GetCollisionObject request with unknown mesh_url[%s]' % req.mesh_resource)
        return res

    def _manage_collision_object_cb(self, req, res):
        """Service callback for ManageCollisionObject

        Execute various operations on collision objects requested by clients
        """
        self.get_logger().info('received service request[op=%d]' % req.op)

        res.success = True

        try:
            if req.op == ManageCollisionObject.Request.CREATE_OBJECT:
                self._create_object(req.object_type, req.object_id,
                                    req.frame_id, req.pose, req.subframe)
            elif req.op == ManageCollisionObject.Request.REMOVE_OBJECT:
                self._remove_object(req.object_id, req.frame_id)
            elif req.op == ManageCollisionObject.Request.ATTACH_OBJECT:
                res.info = self._get_object_info(req.object_id)
                self._attach_object(req.object_id, req.frame_id, req.leaf_id)
            elif req.op == ManageCollisionObject.Request.DETACH_OBJECT:
                res.info = self._get_object_info(req.object_id)
                self._detach_object(req.object_id, req.frame_id, req.leaf_id)
            elif req.op == ManageCollisionObject.Request.MOVE_OBJECT:
                res.info = self._get_object_info(req.object_id)
                self._move_object(req.object_id,
                                  req.frame_id, req.pose, req.subframe)
            elif req.op == ManageCollisionObject.Request.APPEND_TOUCH_LINKS:
                self._append_or_remove_touch_links(req.object_id,
                                                   req.frame_id, True)
            elif req.op == ManageCollisionObject.Request.REMOVE_TOUCH_LINKS:
                self._append_or_remove_touch_links(req.object_id,
                                                   req.frame_id, False)
            elif req.op == ManageCollisionObject.Request.RESET_TOUCH_LINKS:
                self._reset_touch_links()
            elif req.op == ManageCollisionObject.Request.GET_OBJECT_INFO:
                res.info = self._get_object_info(req.object_id)
            elif req.op == ManageCollisionObject.Request \
                          .GET_ATTACHED_CHILD_OBJECT_INFO:
                res.info = self._get_attached_child_object_info(req.frame_id)
            elif req.op == ManageCollisionObject.Request.ALLOW_COLLISION:
                self._set_collision_allowed(req.object_id, req.frame_id, True)
            elif req.op == ManageCollisionObject.Request.DISALLOW_COLLISION:
                self._set_collision_allowed(req.object_id, req.frame_id, False)
            else:
                raise RuntimeError('unknown operation[%d]' % req.op)
        except Exception as e:
            self.get_logger().error('%s' % e)
            res.success = False

        self.get_logger().info('return service response[op=%d]' % req.op)

        return res

    #
    # Operations
    #
    def _create_object(self, object_type, object_id, frame_id, pose, subframe):
        """Create a new collision object

        The created new collision object is not attached to any links
        and its pose is specified as that of subframe of the object
        with respect to the 'frame_id'.

        Args:
          object_type (str): type of object to be created
          object_id   (str): unique ID of object to identification
          frame_id    (str): reference frame for specifying pose of the object
          pose (geometry_msgs/Pose): pose of 'subframe' w.r.t. 'frame_id'
          subframe    (str): subframe name with which the pose of the object
                             is specified
        """
        obj_props = self._obj_props_dict.get(object_type)
        if obj_props is None:
            raise RuntimeError('unknown object type[%s]' % object_type)

        # Setup a new collision object.
        co = CollisionObject()
        co.id = object_id
        if obj_props.collision_meshes != []:
            co.meshes     = obj_props.collision_meshes
            co.mesh_poses = obj_props.collision_mesh_poses
        else:
            co.primitives      = obj_props.primitives
            co.primitive_poses = obj_props.primitive_poses
        co.subframe_names = obj_props.subframe_names
        co.subframe_poses = obj_props.subframe_poses
        co.operation      = CollisionObject.ADD

        # If the object pose is specified as that of subframe other than
        # 'base_link', convert the given pose to that of 'base_link'.
        # Then compute a transform from 'base_link' to the new parent link.
        frame_id, pose = self._find_base_link_and_pose(frame_id, pose,
                                                       co, subframe)
        co.header.frame_id = frame_id
        co.pose            = pose

        # Create a new collision object.
        self._psi.add_object(co)

        # Create info for this object.
        instance_props = CollisionObjectManager.InstanceProperties(object_type)

        # Create subframe transforms.
        base_link = object_id + '/base_link'
        instance_props.subframe_transforms.append(
            TransformStamped(header=Header(frame_id=frame_id),
                             child_frame_id=base_link,
                             transform=_transform_from_pose(pose)))
        for subframe_name, subframe_pose in zip(obj_props.subframe_names,
                                                obj_props.subframe_poses):
            if subframe_name != 'base_link':
                instance_props.subframe_transforms.append(
                    TransformStamped(
                        header=Header(frame_id=base_link),
                        child_frame_id=object_id + '/' + subframe_name,
                        transform=_transform_from_pose(subframe_pose)))

        # Create new marker IDs if not exit for this object.
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
        with self._lock:
            self._instance_props_dict[object_id] = instance_props

        # Add object to AllowedCollisionMatrix(acm)
        self._set_acm_allowed(object_id, None, False)

        self.get_logger().info("created '%s' of type[%s]"
                               %(co.id, object_type))

    def _remove_object(self, object_id, frame_id):
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

    def _attach_object(self, object_id, parent_link, leaf_id):
        """Attach collision object
        Args:
          object_id   (str):  unique ID of the object to be attached/detached
          parent_link (str):  name of link to be parent of the object
        """
        co = self._get_any_object(object_id)
        if co is None:
            raise RuntimeError("unknown collision object '%s'" % object_id)

        # Make this object root of the tree attached to link.
        old_root_id, old_parent_link = self._rotate_tree(co, leaf_id)

        # If 'parent_link' is a subframe of another collision object,
        # get frame ID its 'base_link'.
        parent_link = _get_base_link(parent_link)

        # Lookup transform from 'base_link' of the current collision object
        # to the parent link.
        Tpo = self._tf2_buffer.lookup_transform(parent_link,
                                                co.id + '/base_link', Time())
        self._instance_props_dict[co.id].subframe_transforms[0] = Tpo

        # If 'parent_link' is a 'base_link' of another object, find its
        # attach link and compute pose of 'co' w.r.t. it.
        attach_link, pose \
            = self._find_attach_link_and_pose(parent_link,
                                              _pose_from_transform(
                                                  Tpo.transform))

        # Attach 'co' and its descendants to 'attach_link' with 'pose'
        # described w.r.t. 'attach_link'.
        self._attach_descendants(co, attach_link,
                                 _pose_matrix(pose) @
                                 tfs.inverse_matrix(_pose_matrix(co.pose)))
        self._append_or_remove_touch_links(old_root_id, old_parent_link, True)

    def _detach_object(self, object_id, parent_link, leaf_id):
        aco = self._get_attached_object(object_id)
        if aco is None:
            raise RuntimeError("unknown attached collision object '%s'"
                               % object_id)

        # Make this object root of the tree attached to link.
        old_root_id, old_parent_link = self._rotate_tree(aco.object, leaf_id)

        # Lookup transform from 'base_link' of the current collision object
        # to the parent link.
        self._instance_props_dict[aco.object.id].subframe_transforms[0] \
            = self._tf2_buffer.lookup_transform(_get_base_link(parent_link),
                                            aco.object.id + '/base_link',
                                            Time())

        # Detach 'aco' from its attach link.
        self._psi.remove_attached_object(name=aco.object.id)
        self.get_logger().info("detached '%s' from '%s'"
                               %(aco.object.id, aco.link_name))

        # Since all child attached objects are connected to the current
        # object 'co', we have to switch their attach links to 'link'.
        co = self._get_object(aco.object.id)
        for child_aco in self._psi.get_attached_objects().values():
            if self._get_parent_id(child_aco.object.id) == co.id:
                self._attach_descendants(child_aco.object, co.header.frame_id,
                                         _pose_matrix(co.pose) @
                                         tfs.inverse_matrix(
                                             _pose_matrix(aco.object.pose)))
        self._append_or_remove_touch_links(old_root_id, old_parent_link, True)

    def _move_object(self, object_id, frame_id, pose, subframe):
        co = self._get_any_object(object_id)
        if co is None:
            raise RuntimeError("unknown collision object '%s'" % object_id)

        # Transform the given pose from 'frame_id' to parent link of 'co'.
        parent_link = self._get_parent_link(co.id)
        pose = _pose_from_matrix(
                   _transform_matrix(
                       self._tf2_buffer.lookup_transform(parent_link, frame_id,
                                                         Time()).transform) @
                   _pose_matrix(pose))

        # Transform the given pose of subframe to that of 'base_link'
        # described w.r.t. 'parent_link' which is a parent link of 'object_id'.
        parent_link, pose = self._find_base_link_and_pose(parent_link, pose,
                                                          co, subframe)
        self._instance_props_dict[co.id].subframe_transforms[0] \
            = TransformStamped(header=Header(frame_id=parent_link),
                               child_frame_id=co.id + '/base_link',
                               transform=_transform_from_pose(pose))
        self._move_descendants(co,
                               _transform_matrix(
                                   self._tf2_buffer.lookup_transform(
                                       co.header.frame_id, parent_link,
                                       Time()).transform) @ \
                               tfs.inverse_matrix(_pose_matrix(co.pose)))

    def _append_or_remove_touch_links(self, object_id, link, append):
        aco = self._get_attached_object(object_id)
        if aco is None:
            return
        touch_links = list(set(aco.touch_links) |
                           set(self._get_touch_links(link))) if append else \
                      list(set(aco.touch_links) -
                           set(self._get_touch_links(link)))
        self._psi.attach_object(aco, touch_links=touch_links)
        self.get_logger().info("protect '%s' attached to '%s' with touch links%s" % (aco.object.id, aco.link_name, aco.touch_links))

    def _reset_touch_links(self):
        for aco in self._psi.get_attached_objects().values():
            self._psi.attach_object(aco,
                                    touch_links=self._get_parent_touch_links(
                                                    aco.object.id))
        self.get_logger().info('reset touch links for all attached collision objects')

    def _get_object_info(self, object_id):
        info = CollisionObjectInfo()
        info.object_id = object_id
        co = self._get_object(object_id)
        if co is None:
            aco = self._get_attached_object(object_id)
            if aco is None:
                raise RuntimeError("unknown collision object '%s'" % object_id)
            info.attach_link = aco.link_name
            info.touch_links = aco.touch_links
            info.pose        = PoseStamped(aco.object.header, aco.object.pose)
        else:
            info.pose = PoseStamped(co.header, co.pose)
        info.object_type = self._instance_props_dict[object_id].type
        info.parent_link = self._get_parent_link(object_id)
        return info

    def _get_attached_child_object_info(self, frame_id):
        for aco in self._psi.get_attached_objects().values():
            if self._get_parent_link(aco.object.id) == frame_id:
                info = CollisionObjectInfo()
                info.object_id   = aco.object.id
                info.attach_link = aco.link_name
                info.touch_links = aco.touch_links
                info.pose        = PoseStamped(aco.object.header,
                                               aco.object.pose)
                info.object_type = self._instance_props_dict[info.object_id] \
                                       .type
                info.parent_link = self._get_parent_link(info.object_id)
                return info
        return None

    def _set_collision_allowed(self, object_id, frame_id, allow):
        if frame_id is None:
            others = None
        else:
            other_id, _ = _decompose_link_name(frame_id)
            others = [other_id] if other_id != '' else \
                     self._get_touch_links(frame_id)
        self._set_acm_allowed(object_id, others, allow=allow)

        # acm = self._psi.get_planning_scene(
        #            PlanningSceneComponents.ALLOWED_COLLISION_MATRIX) \
        #           .allowed_collision_matrix
        # for entry_name, entry_value in zip(acm.entry_names, acm.entry_values):
        #     print('--- %s ---' % entry_name)
        #     for other_name, enabled in zip(acm.entry_names,
        #                                    entry_value.enabled):
        #         if enabled:
        #             print('%s <-> %s' % (entry_name, other_name))

    #
    # Utilities
    #
    def _create_link_geometry(self, mesh_url, mesh_pose, mesh_scale):
        link_geometry = LinkGeometry()
        link_geometry.origin = mesh_pose
        link_geometry.primitive.type = 0  # Mesh
        link_geometry.dimensions = [mesh_scale.x, mesh_scale.y, mesh_scale.z]

    def _rotate_tree(self, co, leaf_id):
        def _inverse_transform(transform):
            return TransformStamped(
                       header=Header(frame_id=transform.child_frame_id),
                       child_frame_id=transform.header.frame_id,
                       transform=_transform_from_matrix(
                           tfs.inverse_matrix(
                               _transform_matrix(transform.transform))))

        # If 'co' is not attached to any links, we have reached root!
        if self._get_attached_object(co.id) is None:
            self._psi.attach_object(co, co.header.frame_id)
            return co.id, self._get_parent_link(co.id)

        # If 'co' is not attached to any other collision object or attached
        # to an object with ID of 'leaf_id', we have reached root!
        parent_co = self._get_any_object(self._get_parent_id(co.id))
        if parent_co is None or parent_co.id == leaf_id:
            return co.id, self._get_parent_link(co.id)

        # Reverse parent-child relation between 'co' and its parent.
        old_root_id, old_parent_link = self._rotate_tree(parent_co, leaf_id)
        self._instance_props_dict[parent_co.id].subframe_transforms[0] \
            = _inverse_transform(
                    self._instance_props_dict[co.id].subframe_transforms[0])
        return old_root_id, old_parent_link

    def _attach_descendants(self, co, attach_link, T):
        # Attach 'co' to 'attach_link'.
        co.header.frame_id = attach_link
        co.pose = _pose_from_matrix(T @ _pose_matrix(co.pose))
        touch_links = self._get_parent_touch_links(co.id)
        self._psi.attach_object(co, attach_link, touch_links)
        self.get_logger().info("attached '%s' to '%s' with touch_links%s"
                               %(co.id, attach_link, touch_links))

        # Since all child attached objects are connected to the current
        # object 'co', we have to switch their attach links to 'attach_link'.
        for child_aco in self._psi.get_attached_objects().values():
            if self._get_parent_id(child_aco.object.id) == co.id:
                self._attach_descendants(child_aco.object, attach_link, T)

    def _move_descendants(self, co, T):
        co.pose = _pose_from_matrix(T @ _pose_matrix(co.pose))
        aco = self._get_attached_object(co.id)
        if aco is None:
            self._psi.add_object(co)
        else:
            self._psi.attach_object(co, aco.link_name, aco.touch_links)

        # Set poses for all child attached objects.
        for child_aco in self._psi.get_attached_objects().values():
            if self._get_parent_id(child_aco.object.id) == co.id:
                self._move_descendants(child_aco.object, T)

    def _find_base_link_and_pose(self, frame_id, pose, co, subframe):
        """
        Args:
          frame_id (str): reference frame for specifying pose of the object
          pose (geometry_msgs/Pose): pose of 'subframe' w.r.t. 'frame_id'
          co (moveit_msgs/CollisionObject): colliion object
          subframe (str): subframe name with which the pose of 'co'
                          is specified
        """
        def _subframe_pose(co, subframe):
            return co.subframe_poses[co.subframe_names.index(subframe)]

        # Convert the given pose of 'subframe' of 'co' to that of 'base_link'.
        pose = _pose_from_matrix(_pose_matrix(pose) @
                                 tfs.inverse_matrix(
                                     _pose_matrix(
                                         _subframe_pose(co, subframe))))

        # Separate the parent link 'frame_id' into object ID and subframe name.
        parent_id, parent_subframe = _decompose_link_name(frame_id)

        # If the parent link 'frame_id' is a subframe of any other collision
        # object, return its 'base_link' and the pose of 'base_link' of 'co'
        # w.r.t. it.
        return (frame_id, pose) if parent_id == '' else \
               (parent_id + '/base_link',
                _pose_from_matrix(
                       _pose_matrix(
                           _subframe_pose(self._get_any_object(parent_id),
                                          parent_subframe)) @
                       _pose_matrix(pose)))

    def _find_attach_link_and_pose(self, frame_id, pose):
        # If 'frame_id' is the 'base_link' of any collision object,
        # return its attach link and convert the given pose from 'frame_id'
        # to the attach link.
        co = self._get_any_object(_decompose_link_name(frame_id)[0])
        return (frame_id, pose) if co is None else \
               (co.header.frame_id,
                _pose_from_matrix(_pose_matrix(co.pose) @ _pose_matrix(pose)))

    def _get_object(self, object_id):
        return self._psi.get_objects([object_id]).get(object_id)

    def _get_attached_object(self, object_id):
        return self._psi.get_attached_objects([object_id]).get(object_id)

    def _get_any_object(self, object_id):
        aco = self._get_attached_object(object_id)
        return self._get_object(object_id) if aco is None else aco.object

    def _get_parent_link(self, object_id):
        return self._instance_props_dict[object_id].parent_link

    def _get_parent_id(self, object_id):
        return _decompose_link_name(self._get_parent_link(object_id))[0]

    def _get_touch_links(self, link):
        object_id, _ = _decompose_link_name(link)
        return self._touch_links.get(link, []) if object_id == '' else \
               [object_id + '/base_link']

    def _get_parent_touch_links(self, object_id):
        return self._get_touch_links(self._get_parent_link(object_id))

    def _generate_marker_id_list(self, n):
        marker_id_list = []
        for i in range(n):
            marker_id_list.append(self._marker_id_min)
            self._marker_id_min += 1
        return marker_id_list

    def _delete_markers_and_subframes(self, object_id):
        instance_props = self._instance_props_dict.get(object_id)
        if instance_props is None:
            self.get_logger().error('unknown object[%s]' % object_id)
            return
        self._marker_pub.publish(
            MarkerArray(markers=[Marker(id=marker.id, action=Marker.DELETE)
                                 for marker in instance_props.markers]))
        with self._lock:
            del self._instance_props_dict[object_id]
        self.get_logger().info("removed '%s'" % object_id)

    def _set_acm_allowed(self, object_id, others, allow):
        acm = self._get_acm()
        if others is None:
            acm.set_allowed(object_id, None, allow=allow)
        else:
            for other in others:
                acm.set_allowed(object_id, other, allow=allow)
        self._apply_acm(acm)

        if others is None:
            self.get_logger().info("%s collision against '%s' in default"
                                   % ('allow' if allow else 'disallow',
                                      object_id))
        else:
            self.get_logger().info("%s collision between '%s' and %s"
                                   % ('allow' if allow else 'disallow',
                                      object_id, str(others)))

    def _get_acm(self):
        return self._get_planning_scene.call(
                   GetPlanningScene.Request(
                       component=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)) \
                   .scene.allowed_collision_matrix

    def _apply_acm(self, acm):
        scene = PlanningScene()
        scene.allowed_collision_matrix = acm
        scene.is_diff = True
        scene.robot_state.is_diff = True
        self._psi.apply_planning_scene(scene)


#########################################################################
#  Entry point                                                          #
#########################################################################
def main():
    try:
        rclpy.init(args=sys.argv)

        node = CollisionObjectManager('collision_object_manager')
        # rclpy.spin(node)
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except Exception as e:
        print('*** Terminate the node due to exception: %s' % e)

if __name__ == '__main__':
    main()
