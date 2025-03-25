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
import rospy
from aist_routines.ur                import URRoutines
from aist_routines.ConveniPickAction import ConveniPick
from aist_msgs.msg                   import (PickOrPlaceResult,
                                             PickOrPlaceFeedback,
                                             ConveniPickAction, ConveniPickGoal)
from aist_utility.compat             import *

######################################################################
#  class ConveniRoutines                                             #
######################################################################
class ConveniRoutines(URRoutines):
    def __init__(self):
        super().__init__()

        self._item_props   = rospy.get_param('~item_props')
        self._conveni_pick = ConveniPick(self)

    @property
    def current_robot_name(self):
        return 'g_bot'

    def run(self):
        axis = 'Y'

        while not rospy.is_shutdown():
            self.print_help_messages()
            print('')

            prompt = '{:>5}:{}>> '.format(axis,
                                          self.format_pose(
                                              self.get_current_pose(
                                                  self.current_robot_name))) \
                     if self.current_robot_name else '>> '
            key = raw_input(prompt)

            try:
                _, axis, _ = self.interactive(key, self.current_robot_name,
                                              axis, 1.0)
            except Exception as e:
                print(e)

    # Interactive stuffs
    def print_help_messages(self):
        super().print_help_messages()
        print('=== Conveni picking commands ===')
        print('  s: Search graspabilities')
        print('  a: Attempt to pick and place')
        print('  A: Repeat attempts to pick and place')
        print('  c: Cancel attempts to pick and place')

    def interactive(self, key, robot_name, axis, speed):
        if key == 's':
            item_id = raw_input('  item id? ')
            if item_id == '':
                item_id = 'default'
            self.search_graspabilities(item_id)
        elif key == 'a':
            item_id = raw_input('  item id? ')
            if item_id == '':
                item_id = 'default'
            self.go_to_named_pose(self.current_robot_name, 'home')
            self._conveni_pick.send_goal(item_id, False, 5, self._done_cb)
        elif key == 'A':
            item_id = raw_input('  item id? ')
            if item_id == '':
                item_id = 'default'
            self._conveni_pick.send_goal(item_id, True, 5, self._done_cb)
        elif key == 'c':
            self._conveni_pick.cancel_goal()
        elif robot_name:
            return super().interactive(key, robot_name, axis, speed)
        return robot_name, axis, speed

    # Commands
    def search_graspabilities(self, item_id):
        item_props = self._item_props[item_id]
        self.graspability_send_goal(item_props['robot_name'], item_id, 0)
        self.camera(item_props['camera_name']).trigger_frame()
        return self.graspability_wait_for_result('workspace_center')

    # Utilities
    def _done_cb(self, state, result):
        rospy.sleep(1)          # Pause required after cancelling arm motion
        self.go_to_named_pose(self.current_robot_name, 'home')
