#! /usr/bin/env python
import rospy
import smach
import smach_ros
import sys

import actionlib
from actionlib_msgs.msg import *

##from move_base_msgs.msg import *
#from strands_navigation_msgs.msg import *

#from geometry_msgs.msg import Polygon
#from geometry_msgs.msg import Point32
#from geometry_msgs.msg import Pose

from viper_ros.setup import Setup
from viper_ros.view_planning  import ViewPlanning
from viper_ros.executive import Executive
from viper_ros.navigation import GoTo
from viper_ros.shutdown import Shutdown

import viper_ros.perception as percept

import numpy
import tf

class ObjectSearchSM(smach.StateMachine):
    def __init__(self, mode):
        smach.StateMachine.__init__(self, outcomes=['succeeded',
                                                    'aborted',
                                                    'preempted'])

        self.userdata.action_completed = False

        self._setup  = Setup()
        self._view_planning   = ViewPlanning()
        self._executive       = Executive()
        self._goto            = GoTo()
        self._shutdown        = Shutdown()
        perception = rospy.get_param('~perception', 'nill')
        if mode == 'object':
            reload (percept)
            self._perception = percept.PerceptionReal()
        elif mode == 'human':
            reload (percept)
            self._perception = percept.PerceptionPeople()
        else:
            reload (percept)
            self._perception = percept.PerceptionNill()

        with self:
            smach.StateMachine.add('Setup', self._setup,
                                   transitions={'succeeded': 'ViewPlanning',
                                                'aborted':'Shutdown',
                                                'preempted':'Shutdown'})

            smach.StateMachine.add('ViewPlanning', self._view_planning,
                                   transitions={'succeeded': 'Executive',
                                                'aborted':'Shutdown',
                                                'preempted':'Shutdown'})

            smach.StateMachine.add('Executive', self._executive,
                                   transitions={'succeeded': 'GoTo',
                                                'no_views': 'Shutdown',
                                                'aborted':'Shutdown',
                                                'preempted':'Shutdown'})

            smach.StateMachine.add('GoTo', self._goto,
                                   transitions={'succeeded': 'Perception',
                                                'aborted':'Executive',
                                                'preempted':'Shutdown'})

            smach.StateMachine.add('Perception', self._perception,
                                   transitions={'succeeded':'Executive',
                                                'found_all_objects':'Shutdown',
                                                'aborted':'Shutdown',
                                                'preempted':'Shutdown'}
                               )

            smach.StateMachine.add('Shutdown', self._shutdown,
                                   transitions={'succeeded':'succeeded',
                                                'aborted':'aborted',
                                                'preempted':'preempted'}
                               )
