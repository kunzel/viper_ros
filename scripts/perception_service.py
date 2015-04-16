#!/usr/bin/env python
import rospy
import json
import tf
from viper_ros.srv import Perceive, PerceiveResponse
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2

################################################################
# Perception service
################################################################

class PerceptionService():
	"A class for simulated perception"

	def __init__(self):
            rospy.init_node('perception_service')
            rospy.loginfo("Started perception service")

            # setting up the service
            self.ser = rospy.Service('/perceive', Perceive, self._service)

            rospy.spin()
            rospy.loginfo("Stopped perception service")

        def camera_cb(self, data):
            obj_list = json.loads(data.data)
            return obj_list

        def _service(self,req):
            rospy.loginfo('Incoming service request')
            res = PerceiveResponse()
            res.objects = []
            res.types   = []
            res.poses   = PoseArray()
            try:
                # Get frame of depth cam
                # rospy.loginfo("Wait for '/head_xtion/depth/points'")
                # pointcloud = rospy.wait_for_message('/head_xtion/depth/points', PointCloud2, timeout=10.0)
                # rospy.loginfo("Received msg from '/head_xtion/depth/points'")
                # print pointcloud.header.frame_id

                # TODO: Transform relative object pose into /map frame 
                
                rospy.loginfo("Wait for /semcam")
                msg = rospy.wait_for_message("/semcam", String, timeout=10.0)
                rospy.loginfo("Received msg from /semcam")
                obj_list = self.camera_cb(msg)
                if len(obj_list) == 0:
                    rospy.loginfo("Nothing perceived")
                for obj_desc in obj_list:
                    rospy.loginfo("Perceived: %s" % obj_desc.get('name'))
                    res.objects.append(obj_desc.get('name'))
                    res.types.append(obj_desc.get('type'))
                    # Note: pose is relative to the camera frame
                    pose = Pose()
                    pose.position.x = obj_desc.get('position')[0]
                    pose.position.y = obj_desc.get('position')[1]
                    pose.position.z = obj_desc.get('position')[2]
                    pose.orientation.x = obj_desc.get('orientation').get('x')
                    pose.orientation.y = obj_desc.get('orientation').get('y')
                    pose.orientation.z = obj_desc.get('orientation').get('z')
                    pose.orientation.w = obj_desc.get('orientation').get('w')
                    res.poses.poses.append(pose)
            except rospy.ROSException, e:
                rospy.logwarn("Failed to get /semcam")

            return res

if __name__ == '__main__':
    PerceptionService()
