#! /usr/bin/env python
import rospy
import smach
import smach_ros
import json

from std_msgs.msg import *
from sensor_msgs.msg import *

from soma_roi_manager.soma_roi import SOMAROIQuery

class Setup(smach.State):
    """
    Setup

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=['soma_map', 'soma_conf', 'roi_id'],
                             output_keys=[])

    def execute(self, userdata):
        # Get ROI from mongoDB
        soma_map  = userdata.soma_map
        soma_conf = userdata.soma_conf
        roi_id   = userdata.roi_id
        soma = SOMAROIQuery(soma_map, soma_conf)
        poly = soma.get_polygon(roi_id)

        # set ROI param 
        polygon = []
        for p in poly.points:
            polygon.append([p.x, p.y])
        rospy.set_param('roi', polygon)
        
        return 'succeeded'

