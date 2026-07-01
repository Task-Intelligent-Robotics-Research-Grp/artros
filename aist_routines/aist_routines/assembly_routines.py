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
from geometry_msgs.msg import (PoseStamped, WrenchStamped, Vector3)
from action_msgs.msg   import GoalStatus
from .base_routines    import BaseRoutines
#from cuda_feature_tracker_3d          import FeatureTrackerClient

######################################################################
#  class AssemblyRoutines                                            #
######################################################################
class AssemblyRoutines(BaseRoutines):
    """Implements assembly routines for aist robot system."""

    def __init__(self, name):
        super().__init__(name)
        self._feature_trackers = {}

    # Interactive stuffs
    def print_help_messages(self):
        super().print_help_messages()
        print('=== Assembly commands ===')
        print('  pt: Pick tool')
        print('  PT: Place tool')
        print('  ps: Pick screw')
        print('  PS: Place screw')
        print('  pp: Pick part')
        print('  PP: Place part')
        print('  fb: Fix base')
        print('  FB: Release base')
        print('  at: Begin approaching target')
        print('  AT: Cancel approaching target action')
        print('  H:  Move all robots to home')
        print('  B:  Move all robots to back')

    def process_command(self, command, robot_name, axis, speed):
        if command == 'pt':
            tool_name = input('  tool name? ')
            self.pick_tool(robot_name, tool_name)
        elif command == 'PT':
            self.place_tool(robot_name)
        elif command == 'ps':
            screw_type = input('  screw type? ')
            self.pick_screw(robot_name, screw_type)
        elif command == 'PS':
            self.place_screw(robot_name)
        elif command == 'pp':
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
        elif command == 'H':
            self.go_to_named_pose('all_bots', 'home')
        elif command == 'B':
            self.go_to_named_pose('all_bots', 'back')
        else:
            return super().process_command(command, robot_name, axis, speed)
        return robot_name, axis, speed

    def switch_camera(self, current_robot_name, new_robot_name,
                      laser_power=16):
        self.camera(current_robot_name + '_camera').laser_power = 0
        self.camera(new_robot_name + '_camera').laser_power = laser_power

    def pick_tool(self, robot_name, tool_name, *, timeout_sec=None):
        if self.gripper(robot_name).name == tool_name:
            return (GoalStatus.STATUS_SUCCEEDED, None)
        elif self.gripper(robot_name).name != \
             self.default_gripper_name(robot_name):
            self.place_tool(robot_name)
        status, result = self.pick_at_frame(robot_name, tool_name,
                                            tool_name + '/base_link',
                                            timeout_sec=timeout_sec)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.set_gripper(robot_name, tool_name)
            #self.ftsensor_reset_bias(robot_name)
        return (status, result)

    def place_tool(self, robot_name, *, timeout_sec=None):
        tool_name            = self.gripper(robot_name).name
        default_gripper_name = self.default_gripper_name(robot_name)
        if tool_name == default_gripper_name:
            return (GoalStatus.STATUS_SUCCEEDED, None)
        self.set_gripper(robot_name, default_gripper_name)
        return self.place_at_frame(robot_name, tool_name,
                                   tool_name + '_holder_link',
                                   subframe_link=tool_name + '/base_link',
                                   timeout_sec=timeout_sec)

    def pick_screw(self, robot_name, screw_type, *, timeout_sec=None):
        tool_name = 'screw_tool_' + screw_type[-2:]
        status, result = self.pick_tool(robot_name, tool_name)
        if status != GoalStatus.STATUS_SUCCEEDED:
            return (status, result)
        feeder_name = 'screw_feeder_' + screw_type[-2:]
        screw_id    = self._screw_id(screw_type)
        status, result = self.pick_at_frame(robot_name, screw_id,
                                            screw_id + '/head',
                                            timeout_sec=timeout_sec)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._generate_screw(screw_type)
        return (status, result)

    def place_screw(self, robot_name, *, timeout_sec=None):
        screw_id = self._grasped_object_id(robot_name)
        if screw_id is None:
            return False
        screw_type  = screw_id.rsplit('_', 1)[0]
        feeder_name = 'screw_feeder_' + screw_type[-2:]
        status, result = self.place_at_frame(robot_name, screw_id,
                                             feeder_name + '_inlet_link',
                                             subframe_link=screw_id + '/tip_link',
                                             timeout_sec=timeout_sec)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.com.remove_object(screw_id)
        return (status, result)

    def pick_part(self, robot_name, part_id, subframe, *, timeout_sec=None):
        if self.gripper(robot_name).name != \
           self.default_gripper_name(robot_name):
            self.place_tool(robot_name)
        return self.pick_at_frame(robot_name, part_id,
                                  part_id + '/' + subframe,
                                  timeout_sec=timeout_sec)

    def place_part(self, robot_name, part_id, subframe, place_frame,
                   *, timeout_sec=None):
        if self.gripper(robot_name).name != \
           self.default_gripper_name(robot_name):
            return False
        return self.place_at_frame(robot_name, part_id, place_frame,
                                   subframe_link=part_id + '/' + subframe,
                                   timeout_sec=timeout_sec)

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
        target_wrench.wrench.force  = Vector3(*target_force)
        target_wrench.wrench.torque = Vector3(0, 0, 0)
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
        self.com.reset_touch_links()

    # def _initialize_collision_objects(self, *, timeout_sec=None):
    #     super()._initialize_collision_objects(timeout_sec=timeout_sec)
    #     self._screw_m3_id = 0
    #     self._screw_m4_id = 0
    #     self._generate_screw('screw_m3')
    #     self._generate_screw('screw_m4')

    def _grasped_object_id(self, robot_name):
        gripper_name = self.gripper(robot_name).name
        gripper_link \
            = gripper_name + '_tip_link' \
              if gripper_name == self.default_gripper_name(robot_name) else \
              gripper_name + '/base_link'
        info = self.com.get_child_object_info(gripper_link)
        return info.object_id if info is not None else None

    def _generate_screw(self, screw_type):
        if screw_type == 'screw_m3':
            self._screw_m3_id += 1
            screw_name  = screw_type + '_' + str(self._screw_m3_id)
        else:
            self._screw_m4_id += 1
            screw_name  = screw_type + '_' + str(self._screw_m4_id)
        feeder_name = 'screw_feeder_' + screw_type[-2:]
        self.com.create_object(screw_type,
                               self.pose_from_xyzrpy(
                                   frame_id=feeder_name + '_outlet_link'),
                               object_id=self._screw_id(screw_type))
        return screw_name

    def _screw_id(self, screw_type):
        return screw_type + '_' + str(self._screw_m3_id) \
               if screw_type == 'screw_m3' else \
               screw_type + '_' + str(self._screw_m4_id)

    def _print_object_info(self, info):
        print('    object_id:   %s\n    type:        %s\n    parent_link: %s\n    attach_link: %s\n    touch_links: %s\n    pose:\n%s'
              % (info.object_id, info.object_type, info.parent_link,
                 info.attach_link, info.touch_links, info.pose))

    @staticmethod
    def _get_object_id(link_name):
        tokens = link_name.rsplit('/', 1)
        return tokens[0] if len(tokens) == 2 else ''

    @staticmethod
    def _get_subframe(link_name):
        tokens = link_name.rsplit('/', 1)
        return tokens[1] if len(tokens) == 2 else link_name
