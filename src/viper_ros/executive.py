#! /usr/bin/env python
import rospy
import smach
import smach_ros
import json

from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *

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
                             outcomes=['succeeded', 'no_views','aborted', 'preempted'],
                             input_keys= ['views'],
                             output_keys = ['percentage_complete','robot_pose','ptu_state'])
        self.first_call = True

    def execute(self, userdata):
        rospy.loginfo('Executing state %s', self.__class__.__name__)

        # rospy.loginfo('NO EXECUTION => ABORT ACTION')
        # return 'aborted'

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        if self.first_call:
            self.views = userdata.views
            self.plan_length = len(userdata.views)
            self.first_call = False

        if self.views == []:
            rospy.loginfo('View list is empty.')
            return 'no_views'
        else:
            view = self.views.pop(0)
            userdata.percentage_complete  = (float((self.plan_length - len(self.views))) / float(self.plan_length))  * 100
            userdata.robot_pose = view.get_robot_pose()
            userdata.ptu_state = view.get_ptu_state()

        return 'succeeded'

