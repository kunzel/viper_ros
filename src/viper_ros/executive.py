#! /usr/bin/env python
import rospy
import smach
import smach_ros
import json

from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *

from viper_ros.srv import *

from viper.robots.scitos import ScitosRobot

from viper.core.planner import ViewPlanner
from viper.core.plan import Plan
from viper.core.view import View
from viper.core.robot import Robot

class Executive(smach.State):
    """
    Executive

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded','aborted', 'preempted'],
                             input_keys= ['views','has_next_view'],
                             output_keys = ['has_next_view','state','plan_length','percentage_complete'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state %s', self.__class__.__name__)
        userdata.state = self.__class__.__name__
        r = rospy.Rate(1)
        self.views = userdata.views
        self.plan_length = len(userdata.views)
        userdata.plan_length = len(userdata.views)
        userdata.has_next_view = True
        while True:
            if self.preempt_requested(): 
                self.service_preempt()
                return 'preempted'
            if not userdata.has_next_view:
                userdata.percentage_complete = 100.0
                rospy.loginfo('View list is empty.')
                return 'succeeded'
            r.sleep()
        return 'succeeded'
