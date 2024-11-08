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
        self._sleep_time = rospy.get_param('~sleep_time',  2.0)
        self._check      = False
        if not self._check:
            ns = rospy.get_param('~controller_ns',
                                 self._robot_name + '/aist_ftsensor_controller')
            self._take_sample         = rospy.ServiceProxy(ns + '/take_sample',
                                                           Trigger)
            self._compute_calibration = rospy.ServiceProxy(
                                          ns + '/compute_calibration', Trigger)
            self._save_calibration    = rospy.ServiceProxy(
                                          ns + '/save_calibration', Trigger)
            self._clear_samples       = rospy.ServiceProxy(
                                          ns + '/clear_samples', Trigger)

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
        print('  init:  go to initial pose')
        print('  calib: do calibration')
        print('  check: check calibration')

    def interactive(self, key, robot_name, axis, speed):
        if key == 'init':
            self._move_to(self._initpose)
        elif key == 'calib':
            self._check = False
            self.calibrate()
        elif key == 'check':
            self._check = True
            self.calibrate()
        else:
            return super(FTCalibrationRoutines, self) \
                  .interactive(key, robot_name, axis, speed)
        return robot_name, axis, speed

    # Commands
    def calibrate(self):
        self.go_to_named_pose(self._robot_name, 'home', speed=self._speed)
        if not self._check:
            self._clear_samples()

        self._move_to(self._initpose)

        xyzrpy = copy.copy(self._initpose)
        xyzrpy[3] -= 60
        xyzrpy[4] -= 60
        for i in range(12):
            self._move_to(xyzrpy)
            xyzrpy[3] += 10
            xyzrpy[4] += 10

        xyzrpy = copy.copy(self._initpose)
        xyzrpy[3] += 60
        xyzrpy[4] -= 60
        for i in range(12):
            self._move_to(xyzrpy)
            xyzrpy[3] -= 10
            xyzrpy[4] += 10

        if not self._check:
            res = self._compute_calibration()
            print('  compute calibration: %s' % res.message)
            res = self._save_calibration()
            print('  save calibration: %s' % res.message)

        self.go_to_named_pose(self._robot_name, 'home', speed=self._speed)

    def _move_to(self, xyzrpy):
        if not self._move(xyzrpy):
            return False
        rospy.sleep(self._sleep_time)  # Wait for the robot to settle.

        if not self._check:
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
