#! /usr/bin/env python
import roslib; roslib.load_manifest('viper_ros')
import rospy

import threading

import actionlib

#from smach_ros import ActionServerWrapper

from  viper_ros.srv import *
from  viper_ros.msg import *
from  viper_ros.state_machine import ViewPlannerSM

_EPSILON = 0.0001

class ViewPlannerServer:

    _feedback = viper_ros.msg.ViewPlanningFeedback()
    _result   = viper_ros.msg.ViewPlanningResult()
    
    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                viper_ros.msg.ViewPlanningAction,
                                                execute_cb=self.execute_cb,
                                                auto_start = False)

        self._as.start()
        rospy.loginfo('Started action server for view planning')
        self._is_active = False        

        service_name = 'next_view'
        try:
            rospy.wait_for_service(service_name, timeout=1)
        except:
            rospy.loginfo('Setting up service: %s', service_name )
            self.nv_service = rospy.Service(service_name, NextView, self.next_view)
            rospy.loginfo('Service running: %s', service_name )

    def next_view(self, req):
        res = NextViewResponse()
        # if AS has no goal request, ie it is not active return False
        if not self._is_active:
            res.has_next_view = False
            return res
            
        # if current plan is empty, return False AND finish SM
        if self.sm.userdata.views == []:
            self.sm.userdata.has_next_view = False
            res.has_next_view = False
            return res

        # get next view
        view = self.sm.userdata.views.pop(0)
        # calc percentage of completion
        self.sm.userdata.percentage_complete = (float((self.sm.userdata.plan_length - len(self.sm.userdata.views))) / float(self.sm.userdata.plan_length))  * 100
        res.has_next_view = True
        res.robot_pose = view.get_robot_pose()
        res.ptu_state  = view.get_ptu_state()
        return res
        
    def execute_cb(self, goal):
        self._is_active = True
        rospy.loginfo('Received request: %s %s', goal.roi_id, goal.mode)
        
        # helper variables
        r = rospy.Rate(1)
        success = True
        
        # create the state machine
        sm = ViewPlannerSM()
        self.sm = sm
        sm.userdata.num_of_views = rospy.get_param('~num_of_views', 20)
        sm.userdata.current_view = 0

        sm.userdata.percentage_complete = 0

        # set arguments from call
        sm.userdata.mode   = goal.mode
        sm.userdata.roi_id = goal.roi_id

        # set parameters from parameter server
        sm.userdata.soma_map = rospy.get_param('~soma_map',   'bham')
        sm.userdata.soma_conf = rospy.get_param('~soma_conf', 'main')

                
        smach_thread = threading.Thread(target = sm.execute)
        smach_thread.start()
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
            self._feedback = viper_ros.msg.ViewPlanningFeedback()

            if userdata.num_of_views == 0:
                self._feedback.percentage_complete = 0
            else:
                self._feedback.status = userdata.state 
                self._feedback.percentage_complete = userdata.percentage_complete 
                rospy.loginfo('Progress: %s%%' % str(self._feedback.percentage_complete))

            self._as.publish_feedback(self._feedback)
            r.sleep()

        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        else:
            rospy.loginfo('%s: Failed' % self._action_name)
        self._is_active = False

if __name__ == '__main__':
    rospy.init_node('view_planner_server')
    os = ViewPlannerServer('view_planner')
    rospy.spin()
    
    
