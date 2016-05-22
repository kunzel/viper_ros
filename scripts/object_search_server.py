#! /usr/bin/env python
import roslib; roslib.load_manifest('viper_ros')
import rospy
import time

import threading

import actionlib
#from smach_ros import ActionServerWrapper

from  viper_ros.msg import *
from  viper_ros.state_machine import ObjectSearchSM

_EPSILON = 0.0001

class ObjectSearchActionServer:


    _feedback = viper_ros.msg.ObjectSearchFeedback()
    _result   = viper_ros.msg.ObjectSearchResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                viper_ros.msg.ObjectSearchAction,
                                                execute_cb=self.execute_cb,
                                                auto_start = False)

        self._as.start()
        rospy.loginfo('Started action server for object search')


    def execute_cb(self, goal):

        rospy.loginfo('Received request: mode: %s waypoint:%s roi:%s surface_id:%s', goal.mode, goal.waypoint, goal.roi_id, goal.surface_roi_id)

        # helper variables
        r = rospy.Rate(1)
        success = True

        # create the state machine
        sm = ObjectSearchSM(goal.mode)



        sm.userdata.num_of_views = rospy.get_param('~num_of_views', 20)
        sm.userdata.current_view = 0

        sm.userdata.percentage_complete = 0

        sm.userdata.starttime = int(time.time())

        # set arguments from call
        sm.userdata.waypoint = goal.waypoint
        sm.userdata.roi_id = goal.roi_id

        sm.userdata.surface_roi_id = goal.surface_roi_id
        sm.userdata.mode = goal.mode
        sm.userdata.objects = []
        sm.userdata.found_objects = []
        sm.userdata.people_poses = []

        # set parameters from parameter server
        sm.userdata.soma_map = rospy.get_param('~soma_map', 'aloof')
        sm.userdata.soma_conf = rospy.get_param('~soma_conf', 'aloof')


        smach_thread = threading.Thread(target = sm.execute)
        smach_thread.start()

        #outcome = self.agent.execute_sm_with_introspection()
        r.sleep()

        while sm.is_running() and not sm.preempt_requested():
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                sm.request_preempt()
                success = False
                break

            #rospy.loginfo(self.agent.get_sm().get_active_states())
            userdata = sm.userdata

            # get current pose form state machine
            self._feedback = viper_ros.msg.ObjectSearchFeedback()

            if userdata.num_of_views == 0:
                self._feedback.percent_complete = 0
            else:
                self._feedback.percent_complete = userdata.percentage_complete #(float(userdata.current_view) / float(userdata.plan_lentgh)) * 100
                #rospy.loginfo("Percentage complete: %s", self._feedback.percent_complete)

            self._as.publish_feedback(self._feedback)

            r.sleep()

        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._result.found_objects = sm.userdata.found_objects
            self._as.set_succeeded(self._result)
        else:
            rospy.loginfo('%s: Failed' % self._action_name)
            self._result.found_objects = []
            self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('object_search_server')
    os = ObjectSearchActionServer('search_object')
    rospy.spin()
