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
import rclpy, sys, threading
from typing                          import TypeVar, Generic
from rclpy.executors                 import MultiThreadedExecutor
from aist_routines.base_routines     import BaseRoutines
from aist_routines.assembly_routines import AssemblyRoutines
from aist_routines.kitting_routines  import KittingRoutines

T = TypeVar('T')

######################################################################
#  class InteractiveRoutines                                         #
######################################################################
class InteractiveRoutines(Generic[T]):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info('started')

        threading.Thread(target=self.interactive, daemon=True).start()

    def interactive(self):
        if self.com and 'initial_object_config' in self.settings:
            self._initialize_collision_objects(
                self.settings['initial_object_config'])

        arm_name = self.group_names[0]
        axis     = 'Y'
        speed    = 1.0

        # Reset pose
        self.go_to_named_pose(arm_name, "home")
        self.print_help_messages()

        while rclpy.ok():
            current_pose = self.get_current_pose(arm_name)
            prompt = '{:>5}:{}({})@{}>> ' \
                    .format(axis, self.format_pose(current_pose),
                            speed, arm_name)
            key = input(prompt)
            arm_name, axis, speed = super().interactive(key, arm_name,
                                                        axis, speed)

def _main(routines, name):
    rclpy.init(args=sys.argv)
    node = InteractiveRoutines[routines](name)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

######################################################################
#  entry points                                                      #
######################################################################
def interactive():
    _main(BaseRoutines, 'interactive')

def run_assembly():
    _main(AssemblyRoutines, 'assembly')

def run_kitting():
    _main(KittingRoutines, 'kitting')
