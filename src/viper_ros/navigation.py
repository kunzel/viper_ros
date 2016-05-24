#! /usr/bin/env python
import rospy
import smach
import smach_ros
import json
import math

from actionlib import *
from actionlib.msg import *
from scitos_ptu.msg import PtuGotoAction,PtuGotoGoal
from strands_navigation_msgs.msg import MonitoredNavigationAction, MonitoredNavigationGoal

from mongodb_store.message_store import MessageStoreProxy

from std_msgs.msg import *
from sensor_msgs.msg import *

from viper_ros.msg import ViewInfo

from viper.core.view import View

class GoTo(smach.State):
    """
    GoTo action

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=['waypoint','mode','starttime','robot_pose','ptu_state'])

        self.msg_store = MessageStoreProxy(collection='view_stats')
        
        self.nav_client = actionlib.SimpleActionClient('monitored_navigation', MonitoredNavigationAction)
        rospy.loginfo("Wait for monitored navigation server")
        self.nav_client.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Done")

        self.ptu_client = actionlib.SimpleActionClient('SetPTUState', PtuGotoAction)
        rospy.loginfo("Wait for PTU action server")
        self.ptu_client.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Done")

    def execute(self, userdata):
        rospy.loginfo('Executing state %s', self.__class__.__name__)

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        pose = userdata.robot_pose  

 	rospy.loginfo("GOTO: x: %f y: %f", pose.position.x, pose.position.y)
      	goal = MonitoredNavigationGoal()
	goal.action_server = 'move_base'
      	goal.target_pose.header.frame_id = 'map'
      	goal.target_pose.header.stamp = rospy.get_rostime() #rospy.Time.now()
      	goal.target_pose.pose = pose
      	self.nav_client.send_goal(goal)
      	self.nav_client.wait_for_result()
        res = self.nav_client.get_result()
        rospy.loginfo("Result: %s", str(res))
	if res.outcome != 'succeeded':
            vinfo = ViewInfo()
            vinfo.waypoint = userdata.waypoint
            vinfo.map_name  = rospy.get_param('/topological_map_name', "no_map_name")
            vinfo.mode = userdata.mode
            vinfo.starttime = userdata.starttime
            vinfo.timestamp = int(time.time())
            vinfo.robot_pose = userdata.robot_pose
            vinfo.ptu_state = userdata.ptu_state
            vinfo.nav_failure = True
            vinfo.success = False
            vinfo.soma_objs = []
            self.msg_store.insert(vinfo)
            return 'aborted'
        rospy.loginfo("Reached nav goal")

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        ptu_state = userdata.ptu_state
        goal = PtuGotoGoal()
        goal.pan = math.degrees(ptu_state.position[ptu_state.name.index('pan')])
        goal.tilt = math.degrees(ptu_state.position[ptu_state.name.index('tilt')])
        goal.pan_vel = ptu_state.velocity[ptu_state.name.index('pan')] * 100
        goal.tilt_vel = ptu_state.velocity[ptu_state.name.index('tilt')] * 100
        rospy.loginfo("SET PTU: pan: %f tilt: %f", goal.pan, goal.tilt)
        self.ptu_client.send_goal(goal)
        self.ptu_client.wait_for_result()
        rospy.loginfo("Reached ptu goal")

        return 'succeeded'


