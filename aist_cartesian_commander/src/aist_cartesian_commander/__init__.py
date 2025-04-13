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
from actionlib                    import SimpleActionClient
from aist_cartesian_commander.msg import (TrackWithContactAction,
                                          TrackWithContactGoal)

#########################################################################
#  class CartesianCommanderClient                                       #
#########################################################################
class CartesianCommanderClient(SimpleActionClient):
    def __init__(self, server='cartesian_commander'):
        SimpleActionClient.__init__(self, server + '/track_with_contact',
                                    TrackWithContactAction)

        self.wait_for_server()
        # if not self.wait_for_server(timeout=rospy.Duration(10.0)):
        #     rospy.logerr('(CartesianCommanderClient) failed to connect to server[%s]',
        #                   server + '/track_with_contact')
        #     raise
        rospy.loginfo('(CartesianCommanderClient) connected to server[%s]',
                      server + '/track_with_contact')

    # TrackWithContact action stuffs
    def send_goal(self, target_wrench,
                  done_cb=None, active_cb=None, feedback_cb=None):
        SimpleActionClient.send_goal(self,
                                     TrackWithContactGoal(target_wrench),
                                     done_cb=done_cb, active_cb=active_cb,
                                     feedback_cb=feedback_cb)
