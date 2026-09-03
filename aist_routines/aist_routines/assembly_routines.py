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
from geometry_msgs.msg import PoseStamped, WrenchStamped, Vector3
from action_msgs.msg   import GoalStatus
from .base_routines    import BaseRoutines
#from cuda_feature_tracker_3d          import FeatureTrackerClient

#*********************************************************************
#  class AssemblyRoutines                                            *
#*********************************************************************
class AssemblyRoutines(BaseRoutines):
    """Implements assembly routines for aist robot system."""

    def __init__(self, name):
        super().__init__(name)
        self._feature_trackers = {}

    @property
    def screw_types(self):
        return ('screw_m3', 'screw_m4')

    # Interactive stuffs
    def process_command(self, command, robot_name, axis, speed):
        if command == 'pp':
            part_id  = input('  part ID? ')
            subframe = input('  subframe? ')
            if subframe == '':
                subframe = 'default_grasp'
            self.pick_part(robot_name, part_id, subframe, timeout_sec=0.0)
        elif command == 'PP':
            part_id  = input('  part ID? ')
            subframe = input('  subframe? ')
            if subframe == '':
                subframe = 'base_link'
            place_frame = input('  place frame? ')
            self.place_part(robot_name, part_id, subframe, place_frame,
                            timeout_sec=0.0)
        elif command == 'fb':
            self.fix_part('base')
        elif command == 'FB':
            self.release_part('base')

        elif command == 'at':
            pose_name = input('  viewing pose? ')
            if pose_name == '':
                pose_name = 'fasten_screw_m4_ready'
            target_frame = input('  target frame? ')
            if target_frame == '':
                target_frame = 'base/panel_motor_screw_hole_1'
            self.approach_target(robot_name, pose_name, target_frame)
        elif command == 'AT':
            self.cancel_approach_target(robot_name)
        else:
            return super().process_command(command, robot_name, axis, speed)
        return robot_name, axis, speed

    def do_cmds(self, dummy):
        super().do_cmds(dummy)
        print('=== Assembly commands ===')
        print('  ps: Pick/place screw')
        print('  po: Pick object')
        print('  PO: Place object')
        print('  fb: Fix base')
        print('  FB: Release base')
        print('  at: Begin approaching target')
        print('  AT: Cancel approaching target action')

    def do_ps(self, screw_type):
        '''    ps [screw_type]
        Pick a screw of specified type.'''
        if screw_type:
            if screw_type in self.screw_types:
                self.pick_screw(self._robot_name, screw_type)
            else:
                print('  unknown screw type[%s]!' % screw_type)
        else:
            self.place_screw(self._robot_name)

    def complete_ps(self, text, line, ib, ie):
        return BaseRoutines._complete_default(text, line, self.screw_types)

    def do_po(self, subframe):
        """      po <subframe>
        Pick a collision object with grasp pose at the specified subframe."""
        if subframe in self.descendant_frame_ids(self._robot_name):
            self.pick_object(self._robot_name, subframe, timeout_sec=0.0)
        else:
            print('      unknown subframe[%s]' % subframe)

    def complete_po(self, text, line, ib, ie):
        return BaseRoutines._complete_default(text, line,
                                              self.descendant_frame_ids(
                                                  self._robot_name))

    def switch_camera(self, current_robot_name, new_robot_name,
                      laser_power=16):
        self.camera(current_robot_name + '_camera').laser_power = 0
        self.camera(new_robot_name + '_camera').laser_power = laser_power

    # Assembly stuffs
    def initialize_collision_objects(self):
        super().initialize_collision_objects()
        self._screw_m3_id = 0
        self._screw_m4_id = 0
        self._generate_screw('screw_m3')
        self._generate_screw('screw_m4')

    def pick_screw(self, robot_name, screw_type):
        tool_name = 'screw_tool_' + screw_type[-2:]
        status, result = self.pick_tool(robot_name, tool_name)
        if status != GoalStatus.STATUS_SUCCEEDED:
            return (status, result)
        feeder_name = 'screw_feeder_' + screw_type[-2:]
        screw_id    = self._get_screw_id(screw_type)
        status, result = self.pick_at_frame(robot_name, screw_id,
                                            screw_id + '/head')
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._generate_screw(screw_type)
        return (status, result)

    def place_screw(self, robot_name):
        screw_tip_link = next(filter(lambda frame_id:
                                     frame_id.startswith('screw_m') and
                                     frame_id.endswith('/tip_link'),
                                     self.descendant_frame_ids(robot_name)),
                              None)
        print('### screw_tip_link=%s' % screw_tip_link)
        if screw_tip_link is None:
            return (GoalStatus.STATUS_UNKNOWN, None)
        screw_id    = AssemblyRoutines._get_object_id(screw_tip_link)
        screw_type  = screw_id.rsplit('_', 1)[0]
        feeder_name = 'screw_feeder_' + screw_type[-2:]
        status, result = self.place_at_frame(robot_name, screw_id,
                                             feeder_name + '_inlet_link',
                                             eef_link=screw_tip_link)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.com.remove_object(screw_id)
        return (status, result)

    def pick_object(self, robot_name, subframe, *, timeout_sec=None):
        object_id = AssemblyRoutines._get_object_id(subframe)
        return self.pick_at_frame(robot_name, object_id, subframe,
                                  timeout_sec=timeout_sec)

    def place_part(self, robot_name, subframe, place_frame,
                   *, timeout_sec=None):
        object_id = AssemblyRoutines._get_object_id(subframe)
        return self.place_at_frame(robot_name, object_id, place_frame,
                                   eef_link=subframe, timeout_sec=timeout_sec)

    def fix_part(self, part_id, offset=(), subframe='base_link'):
        gripper = self._grippers['base_fixture']
        gripper.grasp()
        self.com.attach_object(part_id, gripper.tip_link)
        self.com.move_object(part_id, self.pose_from_xyzrpy(offset),
                             part_id + '/' + subframe)

    def release_part(self, part_id):
        gripper = self._grippers['base_fixture']
        gripper.release()
        self.com.detach_object(part_id, gripper.tip_link)

    def approach_target(self, robot_name,
                        pose_name, target_frame, target_force=(0, 0, -5)):
        self.go_to_named_pose(robot_name, pose_name)
        self._ur_robots[robot_name].switch_controller(
            'cartesian_compliance_controller')

        gripper_tip_link = self.gripper(robot_name).tip_link
        object_id = AssemblyRoutines._get_object_id(target_frame)
        self.com.allow_collision(object_id, gripper_tip_link)

        feature_names = [gripper_tip_link, target_frame]
        target_wrench = WrenchStamped()
        target_wrench.header.frame_id = robot_name + '_base_link'
        target_wrench.wrench.force  = Vector3(x=target_force[0],
                                              y=target_force[1],
                                              z=target_force[2])
        target_wrench.wrench.torque = Vector3(x=0, y=0, z=0)
        self._feature_trackers[robot_name].send_goal(pose_name, feature_names,
                                                     target_wrench)

    def cancel_approach_target(self, robot_name):
        tracker = self._feature_trackers[robot_name]
        tracker.cancel_goal()
        self._ur_robots[robot_name].switch_controller(
            'scaled_pos_joint_traj_controller')
        if tracker.wait_for_result():
            result = tracker.get_result()
            self.go_to_named_pose(robot_name, result.pose_name)

    # Utilities
    def _generate_screw(self, screw_type):
        if screw_type == 'screw_m3':
            self._screw_m3_id += 1
            screw_id = screw_type + '_' + str(self._screw_m3_id)
        else:
            self._screw_m4_id += 1
            screw_id = screw_type + '_' + str(self._screw_m4_id)
        feeder_name = 'screw_feeder_' + screw_type[-2:]
        self.com.create_object(screw_type,
                               self.pose_from_xyzrpy(
                                   (), frame_id=feeder_name + '_outlet_link'),
                               object_id=self._get_screw_id(screw_type))
        self.com.allow_collision(screw_id, feeder_name + '_outlet_link')
        return screw_id

    def _get_screw_id(self, screw_type):
        return screw_type + '_' + str(self._screw_m3_id) \
               if screw_type == 'screw_m3' else \
               screw_type + '_' + str(self._screw_m4_id)

    @staticmethod
    def _get_object_id(link_name):
        tokens = link_name.rsplit('/', 1)
        return tokens[0] if len(tokens) == 2 else ''
