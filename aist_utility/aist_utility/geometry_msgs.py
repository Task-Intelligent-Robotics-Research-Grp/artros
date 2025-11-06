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
from geometry_msgs.msg import Point, Vector3, Quaternion, Transform

def depths_to_points(camera_info, u, v, d):
    """
    Back-project 2D image points to 3D space using depths
    """
    npoints = len(d)
    xy = cv2.undistortPoints(np.expand_dims(np.array(list(zip(u, v)),
                                                     dtype=np.float32),
                                            axis=0),
                             np.array(camera_info.k).reshape((3, 3)),
                             np.array(camera_info.d))
    xy = xy.ravel().reshape(npoints, 2)
    return [ Point(x=xy[i, 0]*d[i], y=xy[i, 1]*d[i], z=d[i])
             for i in range(npoints) ]

def tuple_from_vector3(v):
    return (v.x, v.y, v.z)

def tuple_from_quaternion(q):
    return (q.x, q.y, q.z, q.w)

def tuple_from_transform(t):
    return (tuple_from_vector3(t.translation),
            tuple_from_quaternion(t.rotation))

def dict_from_point_correspondence(pc):
    return {'source_point': tuple_from_vector3(pc.source_point),
            'image_point':  tuple_from_vector3(pc.image_point)}

def dict_from_point_correspondences(pcs):
    return {'image_frame':     pcs.header.frame_id,
            'camera_name':     pcs.camera_name,
            'reference_frame': pcs.reference_frame,
            'correspondences': [dict_from_point_correspondence(pc)
                                for pc in pcs.correspondences]}

def dict_from_point_correspondences_set(pcss):
    return [dict_from_point_correspondences(pcs)
            for pcs in pcss.correspondences_set]

def dict_from_point_correspondences_sets(pcsss):
    return [dict_from_point_correspondences_set(pcss)
            for pcss in pcsss.correspondences_sets]
