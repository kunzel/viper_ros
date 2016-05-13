#! /usr/bin/env python
import rospy
import smach
import smach_ros
import json

from std_msgs.msg import *
from sensor_msgs.msg import *
from std_srvs.srv import Trigger

from soma_manager.srv import SOMA2QueryObjs, SOMA2QueryObjsRequest
#from soma_msgs.msg import * #from soma_roi_manager.soma_roi import SOMAROIQuery

class Setup(smach.State):
    """
    Setup

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=['soma_map', 'soma_conf', 'roi_id', 'surface_roi_id'],
                             output_keys=[])

    def execute(self, userdata):
        # Get ROI from mongoDB
        soma_map  = userdata.soma_map
        soma_conf = userdata.soma_conf
        roi_id   = userdata.roi_id
        surface_roi_id   = userdata.surface_roi_id

        # here we inform the world_modeling package that a sequence of observations is about to begin
        try:
            begin_observations_trigger = rospy.ServiceProxy('/begin_observations',Trigger)
            begin_observations_trigger()
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s"%e)



        rospy.loginfo("Waiting for soma query service")
        service_name = '/soma2/query_db'
        rospy.wait_for_service(service_name)
        rospy.loginfo("Done")

        try:
            service = rospy.ServiceProxy(service_name, SOMA2QueryObjs)
            req = SOMA2QueryObjsRequest()

            req.query_type = 2 # just get the ROis
            rospy.loginfo("Requesting all ROIs")
            res = service(req)
            rois = res.rois
            rospy.loginfo("Received  rois: %s", len(res.rois))

        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s"%e)

        polygon = []
        surface_polygon = []
        for r in rois:
            if r.roi_id == roi_id:
                rospy.loginfo("Found ROI %s", roi_id)
                for p in r.posearray.poses:
                    polygon.append([p.position.x, p.position.y])
            elif r.roi_id == surface_roi_id:
                rospy.loginfo("Found SURFACE ROI %s", surface_roi_id)
                for p in r.posearray.poses:
                    surface_polygon.append([p.position.x, p.position.y])

        rospy.set_param('roi', polygon)
        rospy.set_param('surface_roi', surface_polygon)
        return 'succeeded'
