#! /usr/bin/env python
import rospy
import smach
import smach_ros
import json

from std_msgs.msg import *
from sensor_msgs.msg import *

from actionlib import *
from actionlib.msg import *
from scitos_ptu.msg import PtuGotoAction,PtuGotoGoal

from std_srvs.srv import Trigger

class Shutdown(smach.State):
    """
    Shutdown

    """
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted', 'preempted'])

        self.ptu_client = actionlib.SimpleActionClient('SetPTUState', PtuGotoAction)
        rospy.loginfo("Wait for PTU action server")
        self.ptu_client.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Done")

    def execute(self, userdata):



        goal = PtuGotoGoal()
        goal.pan = 0.0
        goal.tilt = 0.0
        goal.pan_vel = 60
        goal.tilt_vel = 60
        rospy.loginfo("SET PTU: pan: %f tilt: %f", goal.pan, goal.tilt)
        self.ptu_client.send_goal(goal)
        self.ptu_client.wait_for_result()
        rospy.loginfo("Reached ptu goal")

        # here we inform the world_modeling package that a sequence of observations has finished
        # I'm doing this at the end here because it blocks while the world_modeling system does its processing
        # which could take 5-10 seconds
        try:
            end_observations_trigger = rospy.ServiceProxy('/end_observations',Trigger)
            end_observations_trigger()
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s"%e)

        return 'succeeded'
