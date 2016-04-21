#! /usr/bin/env python
import rospy
import smach
import smach_ros
import json

from std_msgs.msg import *
from sensor_msgs.msg import *
from world_modeling.srv import *

from recognition_srv_definitions.srv import recognize, recognizeResponse, recognizeRequest

#from world_state.observation import MessageStoreObject, Observation, TransformationStore
#from world_state.identification import ObjectIdentification

#from world_state.state import World, Object
#from world_state.report import PointCloudVisualiser #, create_robblog
#import world_state.geometry as geometry

from sensor_msgs.msg import Image, PointCloud2, CameraInfo, JointState
from geometry_msgs.msg import PoseWithCovarianceStamped

import numpy as np


class PerceptionNill(smach.State):
    """
    Perceive the environemt (or not).

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted', 'preempted', 'found_all_objects'],
                             input_keys=['found_objects','objects'],
                             output_keys=['found_objects'])
        self.found_objs = dict()

        
    def execute(self, userdata):
        rospy.loginfo("Perceiving...")
        rospy.sleep(3)

        # init self.found_objs 
        for obj in userdata.objects:
            if obj not in self.found_objs:
                self.found_objs[obj] = False

        # set found objects to true
        rospy.loginfo("*************")
        for obj in userdata.objects:
            import random
            r = random.random()
            if r > 0.999:
                rospy.loginfo("FOUND: %s", obj)
                self.found_objs[obj] = True
            else:
                rospy.loginfo("NOTHING FOUND")
        rospy.loginfo("*************")
        
        found_all_objects = True
        for obj in self.found_objs:
            if self.found_objs[obj]:
                if obj not in userdata.found_objects:
                    userdata.found_objects.append(obj)
            else:
                found_all_objects = False

        if found_all_objects:
            return 'found_all_objects'
        return 'succeeded' # perception succeeded, but not all objects has been found yet


class PerceptionReal (smach.State):
    """ Perceive the environemt. """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted', 'preempted', 'found_all_objects'],
                             input_keys=['found_objects','objects'],
                             output_keys=['found_objects'])

        self.pc_frame = rospy.get_param('~camera', '/head_xtion/depth_registered/points')
        self.obj_list = []

        self.ir_service_name = '/recognition_service/mp_recognition'
        #rospy.loginfo('Wait for service %s', self.ir_service_name)
        #rospy.wait_for_service(self.ir_service_name)

	self.wu_srv_name = "/update_world_model"

	rospy.loginfo('Waiting for service %s', self.wu_srv_name)
	rospy.wait_for_service(self.wu_srv_name)

        try:
           self.ir_service = rospy.ServiceProxy(self.ir_service_name, recognize)
	   self.update_service = rospy.ServiceProxy(self.wu_srv_name, WorldUpdate) 
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)
            
        #self._world = World()
        self.found_objs = dict()
        #self._pcv = PointCloudVisualiser()

    def execute(self, userdata):
        # pass 

        # rospy.loginfo('Executing state %s', self.__class__.__name__)
        # self.obj_list = []


        # if self.preempt_requested():
        #     self.service_preempt()
        #     return 'preempted'


        # # get point cloud
	try:
		rospy.loginfo('Waiting for pointcloud: %s', self.pc_frame)
   		pointcloud = rospy.wait_for_message(self.pc_frame, PointCloud2 , timeout=60.0)
		rospy.loginfo('Got pointcloud')
       		# pass pc to update service
		self.update_service(pointcloud)
	except rospy.ROSException, e:
		rospy.logwarn("Failed to get %s" % self.pc_frame)
		return 'aborted'
        # DEFAULT_TOPICS = [("/amcl_pose", PoseWithCovarianceStamped),
        #           ("/head_xtion/rgb/image_color", Image), 
        #           ("/head_xtio	n/rgb/camera_info", CameraInfo), 
        #           (self.pc_frame, PointCloud2),
        #           ("/head_xtion/depth/camera_info", CameraInfo),
        #           ("/ptu/state", JointState)]

        # observation =  Observation.make_observation(DEFAULT_TOPICS)
        # rospy.loginfo('Waiting for pointcloud: %s', self.pc_frame)
        # pointcloud = observation.get_message(self.pc_frame)
        # rospy.loginfo('Got pointcloud')
        # tf =  TransformationStore.msg_to_transformer(observation.get_message("/tf"))


        # if self.preempt_requested():
        #     self.service_preempt()
        #     return 'preempted'

        # try:
        #     req = recognizeRequest()
        #     req.cloud = pointcloud
        #     req.complex_result.data = True
        #     rospy.loginfo('Calling service %s' %  self.ir_service_name)
        #     res = self.ir_service(req)
        #     rospy.loginfo('Received result from %s' % self.ir_service_name)
        # except rospy.ServiceException, e2:
        #     rospy.loginfo("Service call failed: %s", e2)
        #     return 'aborted'

        # ################################################################################
        # # Store result into mongodb_store
        # # depth_to_world = tf.lookupTransform("/map", pointcloud.header.frame_id, 
        # #                                     pointcloud.header.stamp)
        # # print depth_to_world
        # # depth_to_world = geometry.Pose(geometry.Point(*(depth_to_world[0])),
        # #                                geometry.Quaternion(*(depth_to_world[1])))
        
        # objects = res.ids
        # print "=" * 80
        # print "Perceived objects"
        # print "=" * 80, "\n"
        # for i in range(len(res.ids)):

        #     rospy.loginfo("Recognized: %s %s" % (res.ids[i], res.confidence[i]))
        #     self.obj_list.append(res.ids[i].data.strip('.pcd'))
        #     new_object =  self._world.create_object()

        #     # TODO
        #     # add objects to waypoint!!!

        #     # cloud (map frame!)
        #     # bbox (map frame!)
        #     # pose (map frame!)

        #     new_object._point_cloud =  MessageStoreObject.create(res.models_cloud[i]) # cloud is in camera frame
        #     new_object.add_observation(observation)
        #     # info about object model and confidence (use: identification?)
        #     new_object.add_identification("SingleViewClassifier",
        #                                   ObjectIdentification({res.ids[i].data.strip('.pcd') : res.confidence[i]}))

        #     # The position (centroid)
        #     print "=" * 10, "\n", res.centroid[i]
        #     position = geometry.Point.from_ros_point32(res.centroid[i])
        #     print "-" * 10, "\n", position
        #     #position.transform(depth_to_world)
        #     # print "-" * 10, "\n", position
        #     pose = geometry.Pose(position)
        #     new_object.add_pose(pose) # no orientation
        #     print "-" * 80, "\n"

        #     bbox_array =  []
        #     for pt in res.bbox[i].point:
        #         p =  geometry.Point.from_ros_point32(pt)
        #         #p.transform(depth_to_world)
        #         bbox_array.append([p.x, p.y, p.z])
        #     bbox = geometry.BBoxArray(bbox_array)
        #     new_object._bounding_box = bbox
            
        # print "=" * 80, "\n"


        # # init self.found_objs 
        # for obj in userdata.objects:
        #     if obj not in self.found_objs:
        #         self.found_objs[obj] = False

        # # set found objects to true
        # for obj in self.obj_list:
        #     self.found_objs[obj] = True

        # found_all_objects = True
        # for obj in self.found_objs:
        #     if self.found_objs[obj]:
        #         if obj not in userdata.found_objects:
        #             userdata.found_objects.append(obj)
        #     else:
        #         found_all_objects = False

        # if found_all_objects:
        #     return 'found_all_objects'
        return 'succeeded' # perception succeeded, but not all objects has been found yet

