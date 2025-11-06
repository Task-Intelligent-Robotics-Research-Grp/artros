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

######################################################################
#  global functions                                                  #
######################################################################
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

def dict_from_camera_info(cinfo, camera_name):
    return {'camera_name':             camera_name,
            'image_height':            cinfo.height,
            'image_width':             cinfo.width,
            'distortion_model':        cinfo.distortion_model,
            'distortion_coefficients': {'rows': 1, 'cols': len(cinfo.d),
                                        'data': [d for d in cinfo.d]},
            'camera_matrix':           {'rows': 3, 'cols': 3,
                                        'data': [k for k in cinfo.k]},
            'rectification_matrix':    {'rows': 3, 'cols': 3,
                                        'data': [r for r in cinfo.r]},
            'projection_matrix':       {'rows': 3, 'cols': 4,
                                        'data': [p for p in cinfo.p]}}
