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
from geometry_msgs.msg                       import PoseStamped
from action_msgs.msg                         import GoalStatus
from aist_tasks.request_help_task            import RequestHelpTask
from aist_tasks.sweep_task                   import SweepTask
from aist_tasks.error_recovery_by_sweep_task import ErrorRecoveryBySweepTask
from .kitting_routines                       import KittingRoutines

#*********************************************************************
#  class HMIRoutines                                                 *
#*********************************************************************
class HMIRoutines(KittingRoutines):
    """ Implements HMI routines for aist robot system.
    """
    def __init__(self, name):
        super().__init__(name)

        self._sweep                   = SweepTask(self)
        self._request_help            = RequestHelpTask(self)
        self._error_recovery_by_sweep = ErrorRecoveryBySweepTask(self)
        self._attempt_bin.server.register_error_handler(
            'pick', self.error_recovery_by_sweep)

    @property
    def sweep_parameters(self):
        return self.settings['sweep_parameters']

    # Interactive stuffs
    def print_help_messages(self):
        super().print_help_messages()
        print('=== HMI commands ===')
        print('  sw: sWeep')
        print('  rh: Request help')

    def process_command(self, command, robot_name, axis, speed):
        if command == 'sw':
            bin_id = 'bin_' + input('  bin id? ')
            self.sweep_bin(robot_name, bin_id, timeout_sec=0.0)
            self.go_to_named_pose(robot_name, 'home')
        elif command == 'rh':
            bin_id = 'bin_' + input('  bin id? ')
            self.request_help_bin(robot_name, bin_id, timeout_sec=0.0)
        else:
            return super().process_command(command, robot_name, axis, speed)
        return robot_name, axis, speed

    # Sweep stuffs
    def sweep_bin(self, robot_name, bin_id, *, timeout_sec=None):
        """ Sweep object in the specified bin.
        Search graspability points from the specified bin and sweep the one
        with the highest score.

        Args:
          robot_name: Robot name.
          bin_id:     ID of bin.

        Returns:
          Tuple of GoalStatus and result of sweeping motion.
        """
        part_id = self.bin_props[bin_id]['part_id']

        # Search for graspabilities.
        status, result = self.search_bin(
                             bin_id,
                             self.fine_graspability_parameters[part_id])

        # Attempt to sweep the item along y-axis.
        pose = PoseStamped(header=result.graspabilities.poses.header,
                           pose=result.graspabilities.poses.poses[0])
        return self.sweep(robot_name, pose, part_id, timeout_sec=timeout_sec)

    def sweep(self, robot_name, pose, part_id, *, timeout_sec=None):
        """ Sweep the specified part along Y-axis of the specified pose.

        Args:
          robot_name:  Robot name.
          pose:        Pose of the sweep point.
          part_id:     ID of part.
          timeout_sec: Timeout time waiting for the result. Seconds to wait,
            if positive. Wait forever, if `None`.

        Returns:
          Tuple of GoalStatus and result of sweeping motion.
        """
        params = self.sweep_parameters[part_id]
        return self._sweep.send_goal(robot_name, pose,
                                     params['sweep_length'],
                                     params['sweep_offset'],
                                     params['approach_offset'],
                                     params['departure_offset'],
                                     params['speed_fast'],
                                     params['speed_slow'],
                                     timeout_sec=timeout_sec)

    # Request help stuffs
    def request_help_bin(self, robot_name, bin_id, *, timeout_sec=None):
        """ Request help for the object in the specified bin.
        Search graspability points from the specified bin and request help
        for the one with the highest score.

        Args:
          robot_name:  Robot name.
          bin_id:      ID of bin.
          timeout_sec: Timeout time waiting for the result. Seconds to wait,
            if positive. Wait forever, if `None`.

        Returns:
          Tuple of GoalStatus and result of sweeping motion.
        """
        part_id = self.bin_props[bin_id]['part_id']

        # Search for graspabilities and select the first one.
        _, result = self.search_bin(bin_id,
                                    self.fine_graspability_parameters[part_id])
        pose = PoseStamped(header=result.graspabilities.poses.header,
                           pose=result.graspabilities.poses.poses[0])

        return self.request_help(robot_name, pose, part_id,
                                 'Please_specify_sweep_direction.',
                                 timeout_sec=timeout_sec)

    def request_help(self, robot_name, pose, part_id, message,
                     *, timeout_sec=None):
        """ Request help for the specified part.

        Args:
          robot_name:  Robot name.
          pose:        Pose of the pick point where the error occured.
          part_id:     ID of part.
          message:     Message sent to the remote operator.
          timeout_sec: Timeout time waiting for the result. Seconds to wait,
            if positive. Wait forever, if `None`.

        Returns:
          Tuple of GoalStatus and result of sweeping motion.
        """
        return self._request_help.send_goal(robot_name, pose, part_id, message,
                                            timeout_sec=timeout_sec)

    def error_recovery_by_sweep(self, goal, stage, pose):
        if stage == 'pick/verify':
            part_id = self.bin_props[goal.bin_id]['part_id']
            status, result = self._error_recovery_by_sweep \
                                 .send_goal(goal.robot_name, pose, part_id,
                                            'Pick_failed!')
            return status != GoalStatus.STATUS_SUCCEEDED
        else:
            return True
