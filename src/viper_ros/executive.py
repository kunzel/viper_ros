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
                             output_keys = ['has_next_view','state','plan_length','percentage_complete','robot_pose','ptu_state'])

        self.next_view_requested = False
        self.percentage_complete = 0

    #     service_name = 'next_view'
    #     try:
    #         rospy.wait_for_service(service_name, timeout=1)
    #     except:
    #         rospy.loginfo('Setting up service: %s', service_name )
    #         self.nv_service = rospy.Service(service_name, NextView, self.next_view)
    #         rospy.loginfo('Service running: %s', service_name )

    # def next_view(self, req):
    #     res = NextViewResponse()
    #     if self.views == []:
    #         self.has_next_view = False
    #         res.has_next_view = False
    #         return res


    #     view = self.views.pop(0)
    #     self.percentage_complete  = (float((self.plan_length - len(self.views))) / float(self.plan_length))  * 100
    #     self.next_view_requested = True
    #     res.has_next_view = True
    #     res.robot_pose = view.get_robot_pose()
    #     res.ptu_state  = view.get_ptu_state()
    #     return res
        
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

            if self.next_view_requested:
                userdata.percentage_complete  = self.percentage_complete
                self.has_view_requested = False

            r.sleep()


        return 'succeeded'

