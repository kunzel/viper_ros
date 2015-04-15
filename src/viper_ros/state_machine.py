#! /usr/bin/env python
import rospy
import smach
import smach_ros
import sys

import actionlib
from actionlib_msgs.msg import *

from viper_ros.setup import Setup
from viper_ros.view_planning  import ViewPlanning
from viper_ros.executive import Executive
from viper_ros.shutdown import Shutdown

import numpy
import tf

class ViewPlannerSM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded',
                                                    'aborted',
                                                    'preempted'])

        self.userdata.action_completed = False

        self._setup  = Setup()
        self._view_planning   = ViewPlanning()
        self._executive       = Executive()
        self._shutdown        = Shutdown()

        with self:
            smach.StateMachine.add('Setup', self._setup, 
                                   transitions={'succeeded': 'ViewPlanning',
                                                'aborted':'aborted',
                                                'preempted':'preempted'})

            smach.StateMachine.add('ViewPlanning', self._view_planning, 
                                   transitions={'succeeded': 'Executive',
                                                'aborted':'aborted',
                                                'preempted':'preempted'})

            smach.StateMachine.add('Executive', self._executive, 
                                   transitions={'succeeded': 'Shutdown',
                                                'aborted':'aborted',
                                                'preempted':'preempted'})

            smach.StateMachine.add('Shutdown', self._shutdown, 
                                   transitions={'succeeded':'succeeded',
                                                'aborted':'aborted',
                                                'preempted':'preempted'}
                               )
                        
            
