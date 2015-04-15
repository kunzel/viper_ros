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
                             outcomes=['succeeded', 'aborted', 'preempted'])


    def execute(self, userdata):
        return 'succeeded'

