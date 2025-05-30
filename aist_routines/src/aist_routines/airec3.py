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
from math                         import radians
from controller_manager_msgs.srv  import (ListControllers, SwitchController,
                                          SwitchControllerRequest)
from aist_routines                import AISTBaseRoutines
from aist_routines.MoveBaseClient import MoveBaseClient
from aist_utility.compat          import *

######################################################################
#  class Airec3Routines                                              #
######################################################################
class Airec3Routines(AISTBaseRoutines, MoveBaseClient):
    ControllerTypes = (
        'effort_controllers/ToroboJointTrajectoryController',
        'effort_controllers/ToroboJointImpedanceController',
        'torobo_controllers/ToroboOnlineJointTrajectoryController',
        'torobo_controllers/ToroboOnlineJointImpedanceController',
        'torobo_controllers/CartesianImpedanceController')

    def __init__(self, reference_frame='', eef_step=None,
                 controller_ns='base_controller'):
        AISTBaseRoutines.__init__(self, reference_frame, eef_step)
        MoveBaseClient.__init__(self, controller_ns)

        controller_manager  = 'controller_manager'

        self._list_controllers   = rospy.ServiceProxy(controller_manager
                                                      + '/list_controllers',
                                                      ListControllers)
        self._switch_controller  = rospy.ServiceProxy(controller_manager
                                                      + '/switch_controller',
                                                      SwitchController)

    # Interactive stuffs
    def print_help_messages(self):
        super().print_help_messages()
        print('=== Airec3 specific commands ===')
        print('  ready:       Make current joint group go to ready pose')
        print('  wready:      Make "whole_body" joint group go to ready pose')
        print('  whome:       Make "whole_body" joint group go to home pose')
        print('  move:        Move mobile base by specified displacement')
        print('  fmove:       Move mobile base to specified frame')
        print('  switch:      Switch controller')

    def interactive(self, key, robot_name, axis, speed=1.0):
        if key == 'ready':
            self.go_to_named_pose(robot_name, 'ready')
        elif key == 'wready':
            self.go_to_named_pose('whole_body', 'ready')
        elif key == 'whome':
            self.go_to_named_pose('whole_body', 'home')
        elif key == 'move':
            x     = float(raw_input('  x = '))
            y     = float(raw_input('  y = '))
            theta = radians(float(raw_input('  theta = ')))
            self.move_base(x, y, theta)
        elif key == 'fmove':
            self.move_base_to_frame(raw_input(' frame = '))
        elif key == 'switch':
            controllers = self.list_controllers()
            print('  available controllers:')
            for n, controller in enumerate(controllers):
                if controller.state == 'running':
                    print('   *%2d. %s' % (n, controller.name))
                else:
                    print('    %2d. %s' % (n, controller.name))
            n = int(raw_input('  controller #? '))
            self.switch_controller(controllers[n].name)
        else:
            return super().interactive(key, robot_name, axis, speed)
        return robot_name, axis, speed

    ###
    ###  Switching controller stuffs
    ###
    def list_controllers(self):
        return list(filter(lambda x: x.type in Airec3Routines.ControllerTypes,
                           self._list_controllers().controller))

    def current_controller(self):
        for controller in self.list_controllers():
            if controller.state == 'running':
                return controller
        return None

    def switch_controller(self, controller_name):
        for controller in self.list_controllers():
            if controller.name == controller_name:
                if controller.state == 'running':
                    rospy.logwarn('Already running[%s]', controller_name)
                    return True
                elif controller.state == 'initialized' or \
                     controller.state == 'stopped':
                    current_controller = self.current_controller()
                    req = SwitchControllerRequest()
                    req.start_controllers = [controller_name]
                    req.stop_controllers  = [] if current_controller is None \
                                            else [current_controller.name]
                    req.strictness        = SwitchControllerRequest.STRICT
                    req.start_asap        = True
                    req.timeout           = 1.0
                    res = self._switch_controller(req)
                    rospy.sleep(0.5)
                    if res.ok:
                        rospy.loginfo('Succesfully switched to controller[%s]',
                                      controller_name)
                    else:
                        rospy.logerr('Failed to switch to controller[%s]',
                                      controller_name)
                    return res.ok
                else:
                    rospy.logwarn("Controller state is '%', returning True.",
                                  controller.state)
                    return True
        rospy.logerr('Specified controller[%s] not found', controller_name)
        return False
