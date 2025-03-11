#!/usr/bin/env python
# Copyright (C) 2020-2021, National Institute of Advanced Industrial Science
# and Technology (AIST), TOYOTA MOTOR CORPORATION, Ltd.
#
# Any using, copying, disclosing information regarding the software and
# documentation without permission of the copyright holders are prohibited.
# The software is provided "AS IS", without warranty of any kind, express or
# implied, including all implied warranties of merchantability and fitness.
# In no event shall the authors or copyright holders be liable for any claim,
# damages or other liability, whether in an action of contract, tort or
# otherwise, arising from, out of or in connection with the software or
# the use or other dealings in the software.

import rospy
from actionlib_msgs.msg       import GoalStatus
from geometry_msgs.msg        import TwistStamped, WrenchStamped, Vector3
from aist_cartesian_commander import CartesianCommanderClient
from aist_utility.compat      import *

######################################################################
#  class TestCartesianCommander                                      #
######################################################################
class TestCartesianCommander(CartesianCommanderClient):
    def __init__(self, server='cartesian_commander'):
        CartesianCommanderClient.__init__(self, server)

        self._twist_link = rospy.get_param('~twist_link',
                                           'a_bot_camera_color_optical_frame')
        self._target_twist = TwistStamped()
        self._target_twist.header.frame_id \
            = rospy.get_param('~robot_base_link', 'a_bot_base_link')
        self._target_twist.twist.linear  = Vector3(0, 0, 0)
        self._target_twist.twist.angular = Vector3(0, 0, 0)

        self._target_wrench = WrenchStamped()
        self._target_wrench.header.frame_id \
            = rospy.get_param('~wrench_link', 'a_bot_gripper_tip_link')
        self._target_wrench.wrench.force  = Vector3(0, 0, 0)
        self._target_wrench.wrench.torque = Vector3(0, 0, 0)

        self._twist_pub = rospy.Publisher('~target_twist', TwistStamped,
                                          queue_size=1)
        rospy.Timer(rospy.Duration(0.1), self._publish_cb)

    def run(self):
        while not rospy.is_shutdown():
            print('============ Available commands ============ ')
            print('  t:   Set twist value')
            print('  w:   Set wrench value')
            print('  RET: Start/Stop tracking')
            print('  q:   Quit')

            key = raw_input('>> ')
            if key == 't':
                self._target_twist.twist.linear.z = float(raw_input('  vz: '))
            elif key == 'w':
                self._target_wrench.wrench.force.z = float(raw_input('  fz: '))
            elif key == '':
                if self.get_state() != GoalStatus.ACTIVE:
                    self.send_goal(self._twist_link, self._target_wrench,
                                   feedback_cb=self._feedback_cb)
                else:
                    self.cancel_goal()
            elif key=='q':
                break

    def _publish_cb(self, event):
        self._target_twist.header.stamp = rospy.Time.now()
        self._twist_pub.publish(self._target_twist)

    def _feedback_cb(self, feedback):
        pass

######################################################################
#  global functions                                                  #
######################################################################
if __name__ == '__main__':

    rospy.init_node('test_cartesian_commander')

    client = TestCartesianCommander('cartesian_commander')
    client.run()
