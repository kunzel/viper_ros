#! /usr/bin/env python
import rospy
import smach
import smach_ros
import json

from std_msgs.msg import *
from sensor_msgs.msg import *

class Shutdown(smach.State):
    """
    Shutdown

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted', 'preempted'],
                             output_keys=['state'])


    def execute(self, userdata):
        rospy.loginfo('Executing state %s', self.__class__.__name__)
        userdata.state = self.__class__.__name__
        return 'succeeded'

