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
import cv2
import numpy as np
import tf_transformations as tfs
from geometry_msgs.msg import (Point, Vector3, Quaternion, Pose, Transform,
                               PoseStamped, TransformStamped)


def pose_matrix(pose: Pose)-> np.array:
    return tfs.translation_matrix((pose.position.x,
                                   pose.position.y,
                                   pose.position.z)) \
         @ tfs.quaternion_matrix((pose.orientation.x, pose.orientation.y,
                                  pose.orientation.z, pose.orientation.w))

def pose_from_matrix(T: np.array)-> Pose:
    t = tfs.translation_from_matrix(T)
    q = tfs.quaternion_from_matrix(T)
    return Pose(position=Point(x=t[0], y=t[1], z=t[2]),
                orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))

def pose_from_xyzrpy(xyzrpy)-> Pose:
    t = xyzrpy[0:3]
    q = tfs.quaternion_from_euler(*np.radians(xyzrpy[3:6]))
    return Pose(position=Point(x=t[0], y=t[1], z=t[2]),
                orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))

def xyzrpy_from_pose(pose: Pose):
    return (pose.position.x, pose.position.y, pose.position.z,
            *np.degrees(tfs.euler_from_quaternion((pose.orientation.x,
                                                   pose.orientation.y,
                                                   pose.orientation.z,
                                                   pose.orientation.w))))

def transform_matrix(transform: Transform)-> np.array:
    return pose_matrix(pose_from_transform(transform))

def transform_from_matrix(T: np.array)-> Transform:
    return transform_from_pose(pose_from_matrix(T))

def pose_from_transform(transform: Transform)-> Pose:
    return Pose(position=Point(x=transform.translation.x,
                               y=transform.translation.y,
                               z=transform.translation.z),
                orientation=Quaternion(x=transform.rotation.x,
                                       y=transform.rotation.y,
                                       z=transform.rotation.z,
                                       w=transform.rotation.w))

def transform_from_pose(pose: Pose)-> Transform:
    return Transform(translation=Vector3(x=pose.position.x,
                                         y=pose.position.y,
                                         z=pose.position.z),
                     rotation=Quaternion(x=pose.orientation.x,
                                         y=pose.orientation.y,
                                         z=pose.orientation.z,
                                         w=pose.orientation.w))

def transform_from_xyzrpy(xyzrpy)-> Pose:
    return transform_from_pose(pose_from_xyzrpy(xyzrpy))

def xyzrpy_from_transform(transform: Transform):
    return xyzrpy_from_pose(pose_from_transform(transform))

def format_transform(transform: Transform|TransformStamped)-> str:
    if isinstance(transform, TransformStamped):
        return '{} <= {}: [{:.4f}, {:.4f}, {:.4f}; {:.4f}, {:.4f}. {:.4f}]' \
            .format(transform.header.frame_id, transform.child_frame_id,
                    *xyzrpy_from_transform(transform.transform))
    else:
        return '[{:.4f}, {:.4f}, {:.4f}; {:.4f}, {:.4f}. {:.4f}]' \
            .format(*xyzrpy_from_transform(transform))


def format_pose(pose: Pose|PoseStamped)-> str:
    if isinstance(pose, PoseStamped):
        return "[{:.4f}, {:.4f}, {:.4f}; {:.4f}, {:.4f}. {:.4f}]@'{}'" \
            .format(*xyzrpy_from_pose(pose.pose), pose.header.frame_id)
    else:
        return '[{:.4f}, {:.4f}, {:.4f}; {:.4f}, {:.4f}. {:.4f}]' \
            .format(*xyzrpy_from_pose(pose))

def depths_to_points(camera_info, u, v, d):
    """
    Back-project 2D image points to 3D space using depths
    """
    npoints = len(d)
    xy = cv2.undistortPoints(np.expand_dims(np.array(list(zip(u, v)),
                                                     dtype=np.float32),
                                            axis=0),
                             camera_info.k.reshape((3, 3)),
                             np.array(camera_info.d))
    xy = xy.ravel().reshape(npoints, 2)
    return [ Point(x=xy[i, 0]*d[i], y=xy[i, 1]*d[i], z=d[i])
             for i in range(npoints) ]
