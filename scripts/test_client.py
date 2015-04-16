#!/usr/bin/env python
import math
import rospy
import json
import tf
from viper_ros.srv import Perceive, PerceiveRequest, PerceiveResponse
from viper_ros.srv import NextView, NextViewRequest, NextViewResponse  

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose

from actionlib import *
from actionlib.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from scitos_ptu.msg import PtuGotoAction,PtuGotoGoal
from viper_ros.msg import ViewPlanningAction, ViewPlanningGoal


################################################################
# Viper test client
################################################################
class TestClient():
  "A class for executing plans"
  def __init__(self):
    rospy.init_node('viper_test_client')

    # setup next view service
    rospy.loginfo("Wait for service: next_view")
    rospy.wait_for_service('next_view')
    self.next_view = rospy.ServiceProxy('next_view', NextView)
    rospy.loginfo("Done")

    # set up perception service
    rospy.loginfo("Wait for service: perceive")
    rospy.wait_for_service('perceive')
    self.perceive = rospy.ServiceProxy('perceive', Perceive)
    rospy.loginfo("Done")

    # set up move base client
    self.mb_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Wait for server: move_base")
    self.mb_client.wait_for_server(rospy.Duration(60))
    rospy.loginfo("Done")

    # set up ptu client
    self.ptu_client = actionlib.SimpleActionClient('SetPTUState', PtuGotoAction)
    rospy.loginfo("Wait for server: SetPTUState")
    self.ptu_client.wait_for_server(rospy.Duration(60))
    rospy.loginfo("Done")

    # set up view planner
    self.viper_client = actionlib.SimpleActionClient('view_planner', ViewPlanningAction)
    rospy.loginfo("Wait for server: view_planner")
    self.viper_client.wait_for_server(rospy.Duration(60))
    rospy.loginfo("Done")

    
  def execute(self):

    rospy.loginfo("Calling view planner ...")
    goal = ViewPlanningGoal()
    goal.roi_id = '1'
    self.viper_client.send_goal(goal)
    rospy.loginfo("Plan generated. Ready for execution.")
    rospy.loginfo("Press 'Enter' to continue")
    raw_input()
    rospy.loginfo("Executing plan ...")
        
    objects = []
    try:
      view = self.next_view()
    except rospy.ServiceException as e:
      rospy.logerr("Service failed: %s"%e)
        
    while view.is_active == True:
      # get robot pose
      pose = view.robot_pose
      # call move base action server
      rospy.loginfo("GOTO: x: %f y: %f", pose.position.x, pose.position.y)
      goal = MoveBaseGoal()
      goal.target_pose.header.frame_id = 'map'
      goal.target_pose.header.stamp = rospy.Time.now()
      goal.target_pose.pose = pose
      self.mb_client.send_goal(goal)
      self.mb_client.wait_for_result()

      # get ptu state
      js = view.ptu_state
      # convert angles from radian to degrees
      pan = math.degrees(js.position[js.name.index('pan')])
      tilt = math.degrees(js.position[js.name.index('tilt')])
      # call ptu action server
      rospy.loginfo("SET PTU: pan: %f tilt: %f", pan, tilt)
      goal = PtuGotoGoal()
      goal.pan = pan
      goal.tilt = tilt
      goal.pan_vel = js.velocity[js.name.index('pan')] * 100
      goal.tilt_vel = js.velocity[js.name.index('tilt')] * 100
      self.ptu_client.send_goal(goal)
      self.ptu_client.wait_for_result()

      # run perception at view
      percept = self.perceive()
      rospy.loginfo("PERCEIVED: %s", percept.objects)
      for o in percept.objects:
        if o not in objects:
          objects.append(o)
          
      # wait for user input
      rospy.loginfo("Press 'Enter' to continue")
      raw_input()
      # get next view
      try:
        view = self.next_view()
      except rospy.ServiceException as e:
        rospy.logerr("Service failed: %s"%e)

    rospy.loginfo("Objects found: %s", objects)
    rospy.loginfo("No more views -> send new goal to view planner")
    
if __name__ == '__main__':
  t = TestClient()
  t.execute()
