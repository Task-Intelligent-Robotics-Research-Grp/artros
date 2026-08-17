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
import tf_transformations as tfs
import numpy as np

from math                        import pi, radians, degrees, cos, sin, sqrt
from geometry_msgs.msg           import Quaternion
from aist_graspability.client    import GraspabilityClient
from aist_graspability_msgs.msg  import Border, Point2D
from aist_utility.fileio         import filepath_from_url

from aist_tasks.attempt_bin_task import AttemptBinTask
from .base_routines              import BaseRoutines

#************************************************************************
#  class KittingRoutines                                                *
#************************************************************************
class KittingRoutines(BaseRoutines):
    """Implements kitting routines for aist robot system."""

    def __init__(self, name: str):
        super().__init__(name)

        self._graspability_client = GraspabilityClient(self)
        self._attempt_bin         = AttemptBinTask(self)

    @property
    def bin_props(self):
        return self.settings['bin_props']

    @property
    def part_props(self):
        return self.settings['part_props']

    @property
    def borders(self):
        return self.settings['borders']

    @property
    def graspability_parameters(self):
        return self.settings['graspability_parameters']

    @property
    def fine_graspability_parameters(self):
        return self.settings['fine_graspability_parameters']

    # Interactive stuffs
    def print_help_messages(self):
        super().print_help_messages()

        print('=== Kitting commands ===')
        print('  s:  Search graspabilities with normal parameters')
        print('  sf: Search graspabilities with fine parameters')
        print('  a:  Attempt to pick and place')
        print('  A:  Repeat attempts to pick and place')
        print('  c:  Cancel attempts to pick and place')
        print('  H:  Move all robots to home')
        print('  B:  Move all robots to back')

    def process_command(self, command, robot_name, axis, speed):
        if command == 's':
            bin_id = 'bin_' + input('  bin id? ')
            self.search_bin(bin_id)
        elif command == 'sf':
            bin_id  = 'bin_' + input('  bin id? ')
            part_id = self.bin_props[bin_id]['part_id']
            self.search_bin(bin_id,
                            self.fine_graspability_parameters.get(part_id))
        elif command == 'a':
            bin_id = 'bin_' + input('  bin id? ')
            self.pick_tool(robot_name, 'suction_tool')
            self.go_to_named_pose(robot_name, 'home')
            self._attempt_bin.send_goal(robot_name, bin_id, False, 5)
        elif command == 'A':
            bin_id = 'bin_' + input('  bin id? ')
            self.pick_tool(robot_name, 'suction_tool')
            self.go_to_named_pose(robot_name, 'home')
            self._attempt_bin.send_goal(robot_name, bin_id, True, 5)
        elif command == 'c':
            self._attempt_bin.cancel_goal(robot_name)
        elif command == 'H':
            self.go_to_named_pose('all_bots', 'home')
        elif command == 'B':
            self.go_to_named_pose('all_bots', 'back')
        elif robot_name:
            return super().process_command(command, robot_name, axis, speed)
        return robot_name, axis, speed

    # Commands
    def search_bin(self, bin_id, graspability_parameters=None):
        # Set parameters for searching graspabilities.
        bin_props = self.bin_props[bin_id]
        self._graspability_client.set_parameters(
            graspability_parameters if graspability_parameters else \
            self.graspability_parameters[bin_props['part_id']])

        # Set function for filtering graspabilities.
        if 'min_height' in bin_props and 'max_height' in bin_props:
            max_slant = 0.0 if graspability_parameters else \
                        bin_props.get('max_slant', 45.0)
            self._graspability_client.set_graspability_filter(
                lambda graspabilities, \
                       target_frame=bin_props['name'], \
                       min_height=bin_props['min_height'], \
                       max_height=bin_props['max_height'], \
                       max_slant=max_slant:
                self._graspability_filter(graspabilities, target_frame,
                                          min_height, max_height, max_slant))
        else:
            self._graspability_client.set_graspability_filter(None)

        border     = self.borders[bin_props['border_id']]
        part_props = self.part_props[bin_props['part_id']]

        # Send goal first and then trigger camera frame.
        self._graspability_client.send_goal(
            Border(points=[Point2D(u=p[0], v=p[1]) for p in border]),
            self._grippers[part_props['gripper_name']].type,
            one_shot=True, timeout_sec=0.0)
        self.camera(part_props['camera_name']).trigger_frame()

        return self._graspability_client.wait()

    # Utilities
    def _graspability_filter(self, graspabilities, target_frame,
                             min_height, max_height, max_slant):
        def _pose_filter(pose, min_height, max_height, max_slant):
            def _normalize(x):
                return x / sqrt(np.dot(x, x))

            # Filter out poses whose height is not within the specified range.
            if pose.position.z < min_height or pose.position.z > max_height:
                return None

            T = tfs.quaternion_matrix((pose.orientation.x, pose.orientation.y,
                                       pose.orientation.z, pose.orientation.w))
            normal = T[0:3, 2]      # local Z-axis at the graspability point
            up     = np.array((0, 0, 1))

            # Cosine of angle between the graspability normal and up vector.
            a = np.dot(normal, up)
            b = cos(radians(max_slant))
            if a < b:
                p = sqrt((1.0 - b*b)/(1.0 - a*a))
                q = b - a*p
                R = np.identity(4, dtype=np.float32)
                R[0:3, 2] = p*normal + q*up                   # fixed Z-axis
                R[0:3, 1] = _normalize(np.cross(R[0:3, 2], T[0:3, 0]))
                R[0:3, 0] = np.cross(R[0:3, 1], R[0:3, 2])
                qR = tfs.quaternion_from_matrix(R)
                pose.orientation = Quaternion(x=qR[0], y=qR[1],
                                              z=qR[2], w=qR[3])
            return pose

        # We have to transform the graspabilitiy poses and contact points
        # to reference frame before moving because these are represented
        # w.r.t. camera frame which will change while moving in the case
        # of "eye on hand".
        graspabilities.contact_points = self.transform_points_to_target_frame(
                                            graspabilities.poses.header,
                                            graspabilities.contact_points,
                                            target_frame)
        graspabilities.poses          = self.transform_poses_to_target_frame(
                                            graspabilities.poses, (),
                                            target_frame)

        poses          = []
        gscores        = []
        contact_points = []
        for pose, gscore, contact_point \
            in zip(graspabilities.poses.poses, graspabilities.gscores,
                   graspabilities.contact_points):
            filtered_pose = _pose_filter(pose, min_height, max_height,
                                         max_slant)
            if filtered_pose is not None:
                poses.append(filtered_pose)
                gscores.append(gscore)
                contact_points.append(contact_point)
        graspabilities.poses.poses    = poses
        graspabilities.gscores        = gscores
        graspabilities.contact_points = contact_points
        return graspabilities

    # def _done_cb(self, state, result):
    #     rospy.sleep(1)          # Pause required after cancelling arm motion
    #     if self.current_robot_name:
    #         self.go_to_named_pose(self.current_robot_name, 'home')
    #         rospy.sleep(1)
    #         self.place_tool(self.current_robot_name)
    #         self.go_to_named_pose(self.current_robot_name, 'home')
