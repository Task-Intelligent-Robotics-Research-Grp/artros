#!/usr/bin/env python

import rospy, copy
from math                import radians
from std_srvs.srv        import Trigger
from aist_routines       import AISTBaseRoutines
from aist_utility.compat import *

######################################################################
#  class FTCalibrationRoutines                                       #
######################################################################
class FTCalibrationRoutines(AISTBaseRoutines):
    def __init__(self):
        super(FTCalibrationRoutines, self).__init__()

        self._robot_effector_frame \
                = rospy.get_param('~robot_effector_frame', '')
        self._robot_name = rospy.get_param('~robot_name', 'a_bot')
        self._initpose   = rospy.get_param('~initpose',    [])
        self._speed      = rospy.get_param('~speed',       0.1)

        if rospy.get('~check', False):
            ns = self._robot_name + '/wrench'
            self._take_sample         = rospy.ServiceProxy(
                                          ns + '/take_sample', Trigger)
            self._compute_calibration = rospy.ServiceProxy(
                                          ns + '/compute_calibration', Trigger)
            self._save_calibration    = rospy.ServiceProxy(
                                          ns + '/save_calibration', Trigger)
            self._clear_samples       = rospy.ServiceProxy(
                                          ns + '/clear_samples', Trigger)
        else:
            self._take_sample         = None
            self._compute_calibration = None
            self._save_calibration    = None
            self._clear_samples       = None

    def run(self):
        # Reset pose
        self.go_to_named_pose(self._robot_name, 'home')
        self.print_help_messages()
        print('')

        axis = 'Y'

        while not rospy.is_shutdown():
            prompt = '{:>5}:{}>> '.format(axis,
                                          self.format_pose(
                                              self.get_current_pose(
                                                  self._robot_name)))
            key = raw_input(prompt)

            _, axis, _ = self.interactive(key, self._robot_name, axis,
                                          self._speed)

    # Interactive stuffs
    def print_help_messages(self):
        super(FTCalibrationRoutines, self).print_help_messages()
        print('=== Calibration commands ===')
        print('  calib: do calibration')

    def interactive(self, key, robot_name, axis, speed):
        if key == 'calib':
            self.calibrate()
        else:
            return super(FTCalibrationRoutines, self) \
                  .interactive(key, robot_name, axis, speed)
        return robot_name, axis, speed

    # Commands
    def calibrate(self):
        self.go_to_named_pose(self._robot_name, 'home')
        if self._clear_samples:
            self._clear_samples()

        self.go_to_pose_goal(self._robot_name,
                             self.pose_from_xyzrpy(self._initpose),
                             speed=self._speed,
                             end_effector_link=self._robot_effector_frame)
        if self._take_sample:
            self._take_sample()

        xyzrpy = copy.copy(self._initpose)
        for i in range(9):
            xyzrpy[3] += 5
            self._move_to(xyzrpy)

        xyzrpy = copy.copy(self._initpose)
        for i in range(9):
            xyzrpy[3] -= 5
            self._move_to(xyzrpy)

        xyzrpy = copy.copy(self._initpose)
        for i in range(9):
            xyzrpy[4] += 5
            self._move_to(xyzrpy)

        xyzrpy = copy.copy(self._initpose)
        for i in range(9):
            xyzrpy[4] -= 5
            self._move_to(xyzrpy)

        if self._compute_calibration:
            res = self._compute_calibration()
            print('  compute calibration: %s' % res.message)
            res = self._save_calibration()
            print('  save calibration: %s' % res.message)

        self.go_to_named_pose(self._robot_name, 'home')

    def _move_to(self, xyzrpy):
        if not self._move(xyzrpy):
            return False

        if self._take_sample:
            self._take_sample()
        return True

    def _move(self, xyzrpy):
        pose = self.pose_from_xyzrpy(xyzrpy)
        print('  move to %s' % self.format_pose(pose))
        success = self.go_to_pose_goal(self._robot_name, pose,
                                       speed=self._speed,
                                       end_effector_link=self._robot_effector_frame)
        print('  reached %s' %
              self.format_pose(self.get_current_pose(self._robot_name)))
        return success


######################################################################
#  global functions                                                  #
######################################################################
if __name__ == '__main__':

    rospy.init_node('run_calibration')

    routines = FTCalibrationRoutines()
    routines.run()
