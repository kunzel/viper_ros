#! /usr/bin/env python
import rospy
import smach
import smach_ros
import json

from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *

import viper
from viper.robots.scitos import ScitosRobot

from viper.core.planner import ViewPlanner
from viper.core.plan import Plan
from viper.core.view import View
from viper.core.robot import Robot

class ViewPlanning(smach.State):
    """
    View planning

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=['num_of_views'],
                             output_keys=['views'])
        self.robot_poses_pub = rospy.Publisher('robot_poses', PoseArray, queue_size=100)

    def get_current_view(self):
        try:
            rospy.loginfo("Wait for /robot_pose")
            robotpose_msg = rospy.wait_for_message("/robot_pose", Pose, timeout=10.0)
        except rospy.ROSException, e:
            rospy.logwarn("Failed to get /robot_pose")
            return None

        try:
            rospy.loginfo("Wait for /ptu/state")
            jointstate_msg = rospy.wait_for_message("/ptu/state", JointState, timeout=10.0)
        except rospy.ROSException, e:
            rospy.logwarn("Failed to get /ptu/state")
            return None
    
        # Add current pose to view_costs (key: '-1')
        current_pose = robotpose_msg
        rospy.loginfo("Current pose: %s" % current_pose)
        current_ptu_state = JointState()
        current_ptu_state.name = ['pan', 'tilt']
        current_ptu_state.position = [jointstate_msg.position[jointstate_msg.name.index('pan')],jointstate_msg.position[jointstate_msg.name.index('tilt')]]
        current_view =  viper.robots.scitos.ScitosView(-1, current_pose, current_ptu_state, None) # ptu pose is not needed for cost calculation
        return current_view
        
    def execute(self, userdata):
        robot = ScitosRobot()
        NUM_OF_VIEWS = userdata.num_of_views

        planner = ViewPlanner(robot)
        
        rospy.loginfo('Generate views.')
        views = planner.sample_views(NUM_OF_VIEWS)
        rospy.loginfo('Generate views. Done. (%s views have been generated)' % len(views))

        view_values = planner.compute_view_values(views)
        view_costs = planner.compute_view_costs(views)

        print view_values

        print view_costs
        
        current_view = self.get_current_view()
        if current_view == None:
            return 'aborted'
        
        vcosts = dict()
        for v in views:
            cost = robot.cost(current_view,v)
            vcosts[v.ID] = cost

        view_costs[current_view.ID] = vcosts

        NUM_OF_PLANS = rospy.get_param('~num_of_plans', 10)
        PLAN_LENGTH = rospy.get_param('~plan_length', 10)
        RHO  = rospy.get_param('~rho', 1.0)
        rospy.loginfo("Started plan sampling.")
        plans = planner.sample_plans(NUM_OF_PLANS, PLAN_LENGTH, RHO, views, view_values, view_costs, current_view.ID)
        rospy.loginfo("Stopped plan sampling.")


        plan_values =  planner.compute_plan_values(plans, view_values, view_costs)
        best_plan_id = planner.min_cost_plan(plan_values)

        for p in plans:
            if p.ID == best_plan_id:
                break
            else:
                p = None
                
        if p.ID != best_plan_id:
            print "Something bad has happend!"

            
        userdata.views = p.views
        # visulaize for debug
        robot_poses  = PoseArray()
        robot_poses.header.frame_id = '/map'
        robot_poses.poses = []
        for v in views:
            robot_poses.poses.append(v.get_robot_pose())
            print len(robot_poses.poses)
        self.robot_poses_pub.publish(robot_poses)
        
        return 'succeeded'

