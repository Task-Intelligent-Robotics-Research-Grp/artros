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
import rclpy, collections, copy, numpy as np
import tf_transformations as tfs
from math                         import pi
from geometry_msgs.msg            import (PoseStamped, PointStamped,
                                          Vector3Stamped,
                                          Point, Quaternion, Vector3)
from visualization_msgs.msg       import Marker
from std_msgs.msg                 import ColorRGBA
from aist_msgs.msg                import RequestHelp as RequestHelpMsg, Pointing
from aist_msgs.action             import RequestHelp, Sweep
from task_wrappers.action_client  import SimpleActionClient
from aist_tasks.request_help_task import RequestHelpTask
from aist_tasks.sweep_task        import SweepTask
from .kitting_routines            import KittingRoutines

######################################################################
#  class HMIRoutines                                                 #
######################################################################
class HMIRoutines(KittingRoutines):
    """Implements HMI routines for aist robot system.
    """
    MarkerProps = collections.namedtuple('MarkerProps', 'id, scale, color')
    _marker_props = {
        'finger' : MarkerProps(0, (0.008, 0.008, 0.008), (1.0, 0.0, 0.0, 1.0)),
        'sweep'  : MarkerProps(1, (0.006, 0.014, 0.015), (1.0, 1.0, 0.0, 1.0))
    }

    def __init__(self, name):
        super().__init__(name)

        self._ground_frame             = self.declare_parameter('ground_frame',
                                                                'ground').value
        self._graspability_params_back = None
        self._request_help_client      = RequestHelpTask(self)
        self._sweep_client             = SweepTask(self)

    @property
    def hmi_graspability_parameters(self):
        return self.settings['hmi_graspability_parameters']

    @property
    def sweep_parameters(self):
        return self.settings['sweep_parameters']

    @property
    def using_hmi_graspability_params(self):
        return self._graspability_params_back is not None

    # Interactive stuffs
    def print_help_messages(self):
        super().print_help_messages()
        print('=== HMI commands ===')
        print('  sh: Search graspabilities with HMI parameters')
        print('  sw: sWeep')
        print('  rh: Request help')

    def process_command(self, command, robot_name, axis, speed):
        if command == 'sh':
            bin_id = 'bin_' + input('  bin id? ')
            self.set_hmi_graspability_params(bin_id)
            self.search_bin(bin_id)
            self.restore_original_graspability_params(bin_id)
        elif command == 'sw':
            bin_id = 'bin_' + input('  bin id? ')
            self.sweep_bin(robot_name, bin_id)
            self.go_to_named_pose(robot_name, 'home')
        elif command == 'rh':
            bin_id = 'bin_' + input('  bin id? ')
            self.request_help_bin(robot_name, bin_id)
        else:
            return super().process_command(command, robot_name, axis, speed)
        return robot_name, axis, speed

    # Graspability stuffs
    def search_bin(self, bin_id, *, max_slant=45.0):
        return super().search_bin(
                   bin_id,
                   max_slant=0.0 if self.using_hmi_graspability_params else
                   max_slant)

    def set_hmi_graspability_params(self, bin_id):
        part_id = self.bin_props[bin_id]['part_id']
        if self.using_hmi_graspability_params:
            self.get_logger().warn('already using graspability paramters for HMI demo.')
            return
        self._graspability_params_back \
            = copy.deepcopy(self.graspability_parameters[part_id])
        self.graspability_parameters[part_id] \
            = copy.deepcopy(self.hmi_graspability_parameters[part_id])
        self.get_logger().info('set graspability paramters for HMI demo.')

    def restore_original_graspability_params(self, bin_id):
        print('*** restore_original_graspability_params')
        part_id = self.bin_props[bin_id]['part_id']
        if not self.using_hmi_graspability_params:
            self.get_logger().warn('already using original graspability paramters.')
            return
        self.graspability_parameters[part_id] \
            = copy.deepcopy(self._graspability_params_back)
        self._graspability_params_back = None
        self.get_logger().info('restore original graspability paramters.')

    # Sweep stuffs
    def sweep_bin(self, robot_name, bin_id, *, timeout_sec=None):
        """ Sweep object in the specified bin.
        Search graspability points from the specified bin and sweep the one
        with the highest score.

        Args:
          bin_id: ID of bin.

        Returns:
          Tuple of GoalStatus and result of sweeping motion.
        """
        part_id = self.bin_props[bin_id]['part_id']

        # Search for graspabilities.
        self.set_hmi_graspability_params(bin_id)
        status, result = self.search_bin(bin_id, max_slant=0.0)
        self.restore_original_graspability_params(bin_id)

        # Attempt to sweep the item along y-axis.
        pose = PoseStamped(header=result.graspabilities.poses.header,
                           pose=result.graspabilities.poses.poses[0])
        R    = tfs.quaternion_matrix((pose.pose.orientation.x,
                                      pose.pose.orientation.y,
                                      pose.pose.orientation.z,
                                      pose.pose.orientation.w))
        return self._sweep(robot_name, pose, R[0:3, 1], part_id,
                           timeout_sec=timeout_sec)

    def _sweep(self, robot_name, target_pose, sweep_dir, part_id,
               *, timeout_sec=None):
        R = tfs.quaternion_matrix((target_pose.pose.orientation.x,
                                   target_pose.pose.orientation.y,
                                   target_pose.pose.orientation.z,
                                   target_pose.pose.orientation.w))
        nz = R[0:3, 2]
        ny = sweep_dir - nz * np.dot(nz, sweep_dir)
        R[0:3, 1] = ny/np.linalg.norm(ny)
        R[0:3, 0] = np.cross(R[0:3, 1], nz)
        q = tfs.quaternion_from_matrix(R)
        target_pose.pose.orientation = Quaternion(x=q[0], y=q[1],
                                                  z=q[2], w=q[3])
        params = self.sweep_parameters[part_id]
        return self._sweep_client.send_goal(robot_name, target_pose,
                                            params['sweep_length'],
                                            params['sweep_offset'],
                                            params['approach_offset'],
                                            params['departure_offset'],
                                            params['speed_fast'],
                                            params['speed_slow'],
                                            timeout_sec=timeout_sec)

    # Request help stuffs
    def request_help_bin(self, robot_name, bin_id, *, timeout_sec=None):
        """
        Search graspability points from the specified bin and request
        finger direction for computing direction to sweep the one with the
        highest score. Computed sweep direction is then visualized.

        Args:
          bin_id: ID specifying the bin
        """
        part_id = self.bin_props[bin_id]['part_id']
        message = '[Request_testing]_Please_specify_sweep_direction.'

        # Search for graspabilities.
        self.set_hmi_graspability_params(bin_id)
        status, result = self.search_bin(bin_id, max_slant=0.0)
        self.restore_original_graspability_params(bin_id)

        # Select the first graspability.
        pose = PoseStamped(header=result.graspabilities.poses.header,
                           pose=result.graspabilities.poses.poses[0])

        # Send request and receive response.
        return self._request_help_client.send_goal(robot_name, pose,
                                                   part_id, message,
                                                   timeout_sec=timeout_sec)

    def request_help_and_sweep(self, robot_name, pose, part_id):
        """
        Request finger direction for the specified graspability point
        and perform sweeping the point in the direction computed from
        the received response.

        Args:
          robot_name: Name of the robot.
          pose:       Pose of the graspability point to be sweeped.
          part_id:    ID for specifying part.

        Returns:
          bool:       `False` if picking task should be aborted.
        """
        self.go_to_named_pose(robot_name, 'sweep_ready')

        message  = 'Picking_failed!'
        status, result = self._request_help_client.send_goal(robot_name, pose,
                                                             part_id, message)

        if result.response.pointing_state == Pointing.SWEEP_RES:
            self.lobber.info('(hmi_demo) Sweep direction given.')
            sweep_dir = self._compute_sweep_dir(pose, result.response)
            self._publish_marker('sweep', pose.header, pose.pose.position,
                                 Vector3(x=sweep_dir[0],
                                         y=sweep_dir[1],
                                         z=sweep_dir[2]))

            status, result = self._sweep(robot_name, pose, sweep_dir, part_id)

            self.go_to_named_pose(robot_name, 'sweep_ready')

            if result.stage == 'departure':
                return False
            elif status == SweepResult.PREEMPTED:
                self.get_logger().warn('(hmi_demo) Preempted while sweeping!')
            elif status != SweepResult.SUCCESS:
                message = 'Planning_for_sweep_failed!'
        elif response.pointing_state == Pointing.RECAPTURE_RES:
            rospy.loginfo('(hmi_demo) Recapture required.')
        else:
            rospy.logwarn('(hmi_demo) Preempted while requesting help!')
        return True

    def cancel_request_help_and_sweep(self):
        self._request_help_clnt.cancel_goal()
        self._sweep_clnt.cancel_goal()

    def _compute_sweep_dir(self, pose: PoseStamped, response: Pointing):
        ppos = pose.pose.position
        fpos = self.transform_points_to_target_frame(response.header,
                                                     [response.point],
                                                     pose.header.frame_id) \
              .point
        sdir = (fpos.x - ppos.x, fpos.y - ppos.y, fpos.z - ppos.z)
        return tuple(sdir / np.linalg.norm(sdir))
