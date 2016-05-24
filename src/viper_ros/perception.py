#! /usr/bin/env python
import rospy
import smach
import smach_ros
import json
import time

import tf
import message_filters
from std_msgs.msg import *
from sensor_msgs.msg import *
from world_modeling.srv import *

from viper_ros.msg import ViewInfo

from recognition_srv_definitions.srv import recognize, recognizeResponse, recognizeRequest

from mongodb_store.message_store import MessageStoreProxy

#from world_state.observation import MessageStoreObject, Observation, TransformationStore
#from world_state.identification import ObjectIdentification

#from world_state.state import World, Object
#from world_state.report import PointCloudVisualiser #, create_robblog
#import world_state.geometry as geometry

from bayes_people_tracker.msg import PeopleTracker
from sensor_msgs.msg import Image, PointCloud2, CameraInfo, JointState
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PoseArray, Pose
from soma_manager.srv import SOMA2InsertObjs
from soma2_msgs.msg import SOMA2Object
from vision_people_logging.srv import CaptureUBD

import math
import itertools
import numpy as np
from scipy.spatial.distance import euclidean
import matplotlib.path as mathpath


# Implementation of Shoelace formula
# http://stackoverflow.com/questions/24467972/calculate-area-of-polygon-given-x-y-coordinates
def poly_area(x, y):
    return 0.5 * np.abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))


# Finding the right polygon in case the lists of xs, ys are not properly ordered
def get_polygon(xs, ys):
    if poly_area(np.array(xs), np.array(ys)) == 0.0:
        xs = [
            [xs[0]] + list(i) for i in itertools.permutations(xs[1:])
        ]
        ys = [
            [ys[0]] + list(i) for i in itertools.permutations(ys[1:])
        ]
        areas = list()
        for ind in range(len(xs)):
            areas.append(poly_area(np.array(xs[ind]), np.array(ys[ind])))
        return mathpath.Path(
            np.array(zip(xs[areas.index(max(areas))], ys[areas.index(max(areas))]))
        )
    else:
        return mathpath.Path(np.array(zip(xs, ys)))


class PerceptionPeople(smach.State):

    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succeeded', 'aborted', 'preempted', 'found_all_objects'],
            input_keys=['people_poses', 'percentage_complete', 'waypoint'], output_keys=['people_poses']
        )

        self.is_occupied = False
        self.uuids = list()
        self._ubd_pos = list()
        self._tracker_pos = list()
        self._tracker_uuids = list()
        self._tfl = tf.TransformListener()

        self.subs = [
            message_filters.Subscriber(
                rospy.get_param("~ubd_topic", "/upper_body_detector/bounding_box_centres"),
                PoseArray
            ),
            message_filters.Subscriber(
                rospy.get_param("~tracker_topic", "/people_tracker/positions"),
                PeopleTracker
            )
        ]
        self.duration = rospy.Duration(
            0.5 * rospy.get_param('~time_window', 120) / rospy.get_param('~num_of_views', 20)
        )
        self.pu_srv_name = "/update_person_model"
        try:
            self.person_update_service = rospy.ServiceProxy(self.pu_srv_name, PersonUpdate)
            self.ubd_srv = rospy.ServiceProxy("/vision_logging_service/capture", CaptureUBD)
            self.insert_srv = rospy.ServiceProxy(
                "/soma2/insert_objects", SOMA2InsertObjs
            )
            self.ubd_srv.wait_for_service()
            self.person_update_service.wait_for_service()
            self.insert_srv.wait_for_service()
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)

        self.robot_pose = Pose()
        rospy.Subscriber("/robot_pose", Pose, self.robot_cb, None, 10)

    def robot_cb(self, pose):
        self.robot_pose = pose

    def cb(self, ubd_cent, pt):
        if not self.is_occupied:
            self.is_occupied = True
            self._tracker_uuids = pt.uuids
            self._ubd_pos = self.to_world_all(ubd_cent)
            self._tracker_pos = [i for i in pt.poses]
            self.is_occupied = False

    def to_world_all(self, pose_arr):
        transformed_pose_arr = list()
        try:
            fid = pose_arr.header.frame_id
            for cpose in pose_arr.poses:
                ctime = self._tfl.getLatestCommonTime(fid, "/map")
                pose_stamped = PoseStamped(Header(1, ctime, fid), cpose)
                # Get the translation for this camera's frame to the world.
                # And apply it to all current detections.
                tpose = self._tfl.transformPose("/map", pose_stamped)
                transformed_pose_arr.append(tpose.pose)
        except tf.Exception as e:
            rospy.logwarn(e)
            # In case of a problem, just give empty world coordinates.
            return []
        return transformed_pose_arr

    def execute(self, data):
        rospy.loginfo("Observing persons...")
        xs = [i[0] for i in rospy.get_param('surface_roi', [])]
        ys = [i[1] for i in rospy.get_param('surface_roi', [])]
        region = get_polygon(xs, ys)
        start_time = rospy.Time.now()
        ts = message_filters.ApproximateTimeSynchronizer(
            self.subs, queue_size=5, slop=0.15
        )
        ts.registerCallback(self.cb)
        rospy.sleep(0.5)
        end_time = rospy.Time.now()
        while not self.preempt_requested() and (end_time - start_time).secs <= self.duration.secs:
            if not self.is_occupied:
                self.is_occupied = True
                for i in self._ubd_pos:
                    for ind, j in enumerate(self._tracker_pos):
                        # conditions to make sure that a person is not detected
                        # twice and can be verified by UBD logging, also is inside
                        # the surface (or target) region
                        conditions = euclidean(
                            [i.position.x, i.position.y], [j.position.x, j.position.y]
                        ) < 0.3
                        uuid = self._tracker_uuids[ind]
                        conditions = conditions and uuid not in self.uuids
                        conditions = conditions and region.contains_point([i.position.x, i.position.y])
                        is_near = False
                        index = -1
                        for inde, pose in enumerate(data.people_poses):
                            if euclidean(pose, [i.position.x, i.position.y]) < 0.3:
                                is_near = True
                                index = inde
                                break
                        conditions = conditions and (not is_near)
                        if is_near and index != -1:
                            try:
                                self.ubd_srv()  # try to capture UBD snapshot to be later stored in world observation
                                rospy.sleep(0.5)
                                rospy.loginfo("This person has been detected before, updating world model...")
                                self.person_update_service(id=str(self.uuids[index]), waypoint=data.waypoint)
                            except rospy.ROSException, e:
                                rospy.logerr("Service call failed: %s" % e)
                                return 'aborted'

                        if conditions:
                            self.ubd_srv()  # try to capture UBD snapshot to be later stored in world observation
                            rospy.sleep(0.5)
                            self.uuids.append(uuid)
                            data.people_poses.append([i.position.x, i.position.y])
                            rospy.loginfo(
                                "%d persons have been detected so far..." % len(data.people_poses)
                            )

                            # old code: for reference
                            #human = SOMA2Object()
                            #human.id = self._tracker_uuids[ind]
                            #human.config = rospy.get_param('~soma_conf', "no_config")
                            #human.type = "Human"
                            #human.pose = i
                            #human.sweepCenter = self.robot_pose
                            #human.mesh = "package://soma_objects/meshes/plant_tall.dae"
                            #human.logtimestamp = rospy.Time.now().secs
                            #self.insert_srv([human])

                            # new code, using world_modeling
                            try:
                                self.person_update_service(id=str(self._tracker_uuids[ind]),waypoint=data.waypoint)
                            except rospy.ROSException, e:
                                rospy.logerr("Service call failed: %s" % e)
                                return 'aborted'

                self.is_occupied = False

            end_time = rospy.Time.now()
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
        rospy.loginfo(
            "Time to observe persons is %d seconds" % ((end_time - start_time).secs)
        )
        rospy.loginfo(
            "Exploration percentage so far is %.2f" % float(data.percentage_complete)
        )
        if len(self.uuids) < 1:
            rospy.loginfo("No person is found at this view")
        if float(data.percentage_complete) >= 90.0:
            rospy.loginfo(
                "Total detected persons is %d." % len(data.people_poses)
            )
            return "found_all_objects"
        return "succeeded"


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
                             input_keys=['found_objects','objects', 'waypoint', 'mode', 'starttime', 'robot_pose', 'ptu_state'],
                             output_keys=['found_objects'])

        
        self.msg_store = MessageStoreProxy(collection='view_stats')

        self.pc_frame = rospy.get_param('~camera', '/head_xtion/depth_registered/points')
        self.obj_list = []

        self.ir_service_name = '/recognition_service/mp_recognition'
        #rospy.loginfo('Wait for service %s', self.ir_service_name)
        #rospy.wait_for_service(self.ir_service_name)

	self.wu_srv_name = "/update_world_model"
	self.pu_srv_name = "/update_person_model"

	rospy.loginfo('Waiting for service %s', self.wu_srv_name)
	rospy.wait_for_service(self.wu_srv_name)

        try:
           self.ir_service = rospy.ServiceProxy(self.ir_service_name, recognize)
           self.world_update_service = rospy.ServiceProxy(self.wu_srv_name, WorldUpdate)
           self.person_update_service = rospy.ServiceProxy(self.pu_srv_name, PersonUpdate)
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)

        #self._world = World()
        self.found_objs = dict()
        #self._pcv = PointCloudVisualiser()

    def execute(self, userdata):
        # pass

        rospy.loginfo('Executing state %s', self.__class__.__name__)

        vinfo = ViewInfo()
        vinfo.waypoint = userdata.waypoint
        vinfo.map_name  = rospy.get_param('/topological_map_name', "no_map_name")
        vinfo.mode = userdata.mode
        vinfo.starttime = userdata.starttime
        vinfo.timestamp = int(time.time())
        vinfo.robot_pose = userdata.robot_pose
        vinfo.ptu_state = userdata.ptu_state
        vinfo.nav_failure = False
        
        # get point cloud
        try:
            rospy.loginfo('Waiting for pointcloud: %s', self.pc_frame)
            pointcloud = rospy.wait_for_message(self.pc_frame, PointCloud2 , timeout=60.0)
            rospy.loginfo('Got pointcloud')
            res = self.world_update_service(input=pointcloud,waypoint=userdata.waypoint)

            if len(res.objects) > 0: 
                vinfo.success = True
            else:
                vinfo.success = False
            vinfo.soma_objs = res.objects
            self.msg_store.insert(vinfo)
            
        except rospy.ROSException, e:
            rospy.logwarn("Failed to get %s" % self.pc_frame)
            return 'aborted'

        return 'succeeded' # perception succeeded, but not all objects has been found yet


