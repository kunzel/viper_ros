#! /usr/bin/env python
import rospy
import smach
import smach_ros
import json

from viper_ros.msg import ViewInfo
from mongodb_store.message_store import MessageStoreProxy

from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *

from geometry_msgs.msg import Point32

from viper.srv import GetKeys, GetKeysRequest, SetOctomap, SetOctomapRequest

import viper
from viper.robots.scitos_simple import ScitosRobot

from viper.core.planner import ViewPlanner
from viper.core.plan import Plan
from viper.core.view import View
from viper.core.robot import Robot

from octomap_msgs.msg import Octomap
from semantic_map_publisher.srv import ObservationOctomapServiceRequest, ObservationOctomapService

from soma_pcl_segmentation.srv import GetProbabilityAtViewRequest, GetProbabilityAtView

from visualization_msgs.msg import Marker, InteractiveMarkerControl
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import ColorRGBA

class Pmf(object):

    def __init__(self):
        self.d = dict()

    def prob(self, x):
        if x == -1:
            return 0.0
        return self.d[x]  

    def set(self, x, val):
        self.d[x] = val

    def unset(self, x):
        if x in self.d:
            self.d.pop(x)

    def keys(self):
        return self.d.keys()

    def total(self):
        total = sum(self.d.itervalues())
        return total
        
    def normalize(self):
        total = self.total()
        if total == 0.0:
            return total

        for x in self.d:
            self.d[x] /= total

        return total

    def random(self):
        target = random.random()
        total = 0.0
        for x, p in self.d.iteritems():
            total += p
            if total >= target:
                return x
        return self.keys()[-1]

################################################################
# Ray-casting algorithm
#
# adapted from http://rosettacode.org/wiki/Ray-casting_algorithm
################################################################
_eps = 0.00001

def ray_intersect_seg(p, a, b):
    ''' takes a point p and an edge of two endpoints a,b of a line segment returns boolean
    '''
    if a.y > b.y:
        a,b = b,a
    if p.y == a.y or p.y == b.y:
        p = Point32(p.x, p.y + _eps, 0)

    intersect = False

    if (p.y > b.y or p.y < a.y) or (
        p.x > max(a.x, b.x)):
        return False

    if p.x < min(a.x, b.x):
        intersect = True
    else:
        if abs(a.x - b.x) > sys.float_info.min:
            m_red = (b.y - a.y) / float(b.x - a.x)
        else:
            m_red = sys.float_info.max
        if abs(a.x - p.x) > sys.float_info.min:
            m_blue = (p.y - a.y) / float(p.x - a.x)
        else:
            m_blue = sys.float_info.max
        intersect = m_blue >= m_red
    return intersect

def is_odd(x): return x%2 == 1

def is_inside(p, poly):
    ln = len(poly)
    num_of_intersections = 0
    for i in range(0,ln):
        num_of_intersections += ray_intersect_seg(p, poly[i], poly[(i + 1) % ln])

    return is_odd(num_of_intersections)



class ViewPlanning(smach.State):
    """
    View planning

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=['waypoint','mode', 'num_of_views','objects', 'surface_roi_id'],
                             output_keys=['views'])
        
        self.msg_store = MessageStoreProxy(collection='view_stats')
        self.robot_poses_pub = rospy.Publisher('robot_poses', PoseArray, queue_size=100)
        self.vis = Vis()

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
        current_ptu_state.velocity = [float(1.0), float(1.0)]
        current_ptu_state.effort = [float(1.0), float(1.0)]
        current_view =  viper.robots.scitos.ScitosView(-1, current_pose, current_ptu_state, None) # ptu pose is not needed for cost calculation
        return current_view
        
    def in_roi(self, pose, roi):

        p = Point32(pose.position.x,pose.position.y,0)        
        return is_inside(p, roi.points)

    def get_view_infos(self, waypoint, map_name, mode, nav_failure, success):

        vinfos = []
        msg_query = { 'waypoint':  waypoint,
                      'map_name':  map_name,
                      'mode':  mode,
                      'nav_failure':  nav_failure,
                      'success':  success
        }
        try:
            res = self.msg_store.query(ViewInfo._type, msg_query)
        except:
            rospy.logwarn("Failed to get view infos")
            return []
            
        for i, vinfo in enumerate(res):
            print i, vinfo[0].starttime
            novel_view = True
            for vi in vinfos:
                if self.equals(vinfo[0], vi):
                    novel_view= False
                    break
                    
            if novel_view:
                vinfos.append(vinfo[0])
            else:
                print "Ignore view. It is already included!!!"
                
        return vinfos

    def equals(self, v1, v2):

        if v1.robot_pose == v2.robot_pose and v1.ptu_state == v2.ptu_state:
            return True
        return False
        
        

    def execute(self, userdata):
        robot = ScitosRobot()
        MIN_COVERAGE = rospy.get_param('~min_coverage', 0.9)
        NUM_OF_VIEWS = userdata.num_of_views

        
        waypoint = userdata.waypoint
        mode = userdata.mode
        map_name = rospy.get_param('/topological_map_name', "no_map_name")
        nav_failure = False
        success = True        
        vinfos = self.get_view_infos(waypoint, map_name, mode, nav_failure, success) 
        db_views = robot.generate_views_from_view_infos(vinfos)


        surface_roi = rospy.get_param('surface_roi',[])
        points = []
        for point in surface_roi:
            rospy.loginfo('Point: %s', point)
            points.append(Point32(float(point[0]),float(point[1]),0))

        surface_polygon = Polygon(points) 

        octomap = Octomap()
        #octomap_service_name = '/Semantic_map_publisher_node/SemanticMapPublisher/ObservationOctomapService'
        rospy.loginfo("Waiting for semantic map service")
        octomap_service_name = '/semantic_map_publisher/SemanticMapPublisher/ObservationOctomapService'
        rospy.wait_for_service(octomap_service_name)
        rospy.loginfo("Done")
        try:
            octomap_service = rospy.ServiceProxy(octomap_service_name, ObservationOctomapService)
            req = ObservationOctomapServiceRequest()
            req.waypoint_id = userdata.waypoint
            req.resolution = 0.05
            rospy.loginfo("Requesting octomap from semantic map service")
            res = octomap_service(req)
            octomap = res.octomap
            rospy.loginfo("Received octomap: size:%s resolution:%s", len(octomap.data), octomap.resolution)

        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s"%e)
        

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'



        rospy.loginfo("Waiting for octomap set-octomap service")
        service_name = '/set_octomap'
        rospy.wait_for_service(service_name)
        rospy.loginfo("Done")

        try:
            service = rospy.ServiceProxy(service_name, SetOctomap)
            req = SetOctomapRequest()
            req.octomap = octomap
            rospy.loginfo("Requesting to set octomap from set-octomap service")
            res = service(req)
    
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s"%e)
        #####################

            
        rospy.loginfo("Waiting for octomap get-keys service")
        service_name = '/get_keys'
        rospy.wait_for_service(service_name)
        rospy.loginfo("Done")

        voxel_marker = MarkerArray() 
        self.vis.voxel_id = 0
        try:
            service = rospy.ServiceProxy(service_name, GetKeys)
            req = GetKeysRequest()
            #req.octomap = octomap
            rospy.loginfo("Requesting keys from octomap get-keys service")
            res = service(req)

            #octomap_keys = res.keys
            octomap_keys = []
            # JUST USE THE KEYS THAT ARE IN THE SURFCE ROI
            for i,pose in enumerate(res.posearray.poses):
                if self.in_roi(pose,surface_polygon):
                    self.vis.create_voxel_marker(voxel_marker, pose)
                    octomap_keys.append(res.keys[i])



            rospy.loginfo("Received octomap_keys: size:%s", len(octomap_keys))
            print "OCTOMAP KEYS", octomap_keys
            if len(octomap_keys) == 0:
                rospy.logerr("Abort search. Octomap has no keys.")
                return 'aborted'
                            
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s"%e)
            
        self.vis.pubvoxel.publish(voxel_marker)
    
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        planner = ViewPlanner(robot)
        rospy.loginfo('Generate views.')
        new_views = planner.sample_views_coverage_in_roi(NUM_OF_VIEWS, MIN_COVERAGE, octomap, octomap_keys)
        rospy.loginfo('Generate views. Done. (%s views have been generated)' % len(new_views))
        
        if len(new_views) == 0:
            rospy.logerr("Abort search. View generation failed.")
            return 'aborted'

        views = []
        vs = []
        for v in new_views:
            vs.append(v.ID)
            views.append(v)
        print "NEW views:", len(vs)

        dbvs = []
        for v in db_views:
            dbvs.append(v.ID)
            views.append(v)
        print "DB views:", len(dbvs)
        
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        print "VIEWS SIZE:", len(views)

        view_values = planner.compute_view_values(views, octomap)

        #################################################
        #### TESTING
        #return 'aborted'
        ################################################

        
        # DO NOT USE SEMANTIC SEGMANTATION FOR TSC DEPLOYMENT
        # rospy.loginfo("Waiting for soma pcl segmenation")
        # service_name = '/soma_probability_at_view'
        # rospy.wait_for_service(service_name)
        # rospy.loginfo("Done")

        # view_probs = dict()
        # for v in views:
        #     print v.ID, v.get_keys(), len(v.get_values())
        #     try:
        #         service = rospy.ServiceProxy(service_name, GetProbabilityAtView)
        #         req = GetProbabilityAtViewRequest()
        #         req.waypoint = userdata.waypoint
        #         req.objects = userdata.objects
        #         req.keys = v.get_keys()
        #         rospy.loginfo("Requesting probability for view")
        #         res = service(req)
        #         prob = res.probability
        #         rospy.loginfo("Received probability: %s", res.probability)
        #         # set value of view
        #         view_probs[v.ID] = prob
        #     except rospy.ServiceException, e:
        #         rospy.logerr("Service call failed: %s"%e)

            
        # view_values_vis = view_values  
        # total_sum = sum(view_values.values())
        # for p in view_values:
        #     view_values[p] = float(view_values[p]) / float(total_sum)  
        #     view_values_vis[p] = float(view_values[p]) / float(total_sum) * 100.0

        print view_values
        #print view_probs
        #view_values = view_probs

        view_costs = planner.compute_view_costs(views)
        #print view_costs
        
        current_view = self.get_current_view()
        if current_view == None:
            return 'aborted'
        
        vcosts = dict()
        for v in views:
            cost = robot.cost(current_view,v)
            vcosts[v.ID] = cost
            view_costs[v.ID][current_view.ID] = cost  

        view_costs[current_view.ID] = vcosts
        view_costs[current_view.ID][current_view.ID] = 0


        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        NUM_OF_PLANS = rospy.get_param('~num_of_plans', 10)
        #PLAN_LENGTH = rospy.get_param('~plan_length', 10)

        # TODO: determine BEST_M and/or TIME_WINDOW based on ROI size
        BEST_M = rospy.get_param('~best_m', 10)
        TIME_WINDOW = rospy.get_param('~time_window', 120)

        best_m_given_coverage = min(BEST_M, len(new_views))
        time_window_given_coverage = min(TIME_WINDOW, len(new_views) * 12.0)
        
        RHO  = rospy.get_param('~rho', 1.0)
        rospy.loginfo("Started plan sampling.")

        #plans = planner.sample_plans(NUM_OF_PLANS, PLAN_LENGTH, RHO, views, view_values, view_costs, current_view.ID)
        plans = planner.sample_plans_IJCAI(NUM_OF_PLANS, time_window_given_coverage, RHO, best_m_given_coverage, views, view_values, view_costs, current_view, current_view)

        
        rospy.loginfo("Stopped plan sampling.")

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'


        plan_values =  planner.compute_plan_values(plans, view_values, view_costs)

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        best_plan_id = planner.min_cost_plan(plan_values)


        for p in plans:
            if p.ID == best_plan_id:
                pids = []
                for v in p.views:
                    pids.append(v.ID)
                self.vis.visualize_plan(p, plan_values)
                # frustum marker
                # call compute_values to calc the frustum
                frustum_marker = MarkerArray()    
                idx = 0
                for view in views:
                    if view.ID in pids and view.ID != -1:
                        val = view_values[view.ID]
                        print idx, val
                        if val > 0.0:
                            print "Create frustum marker with value", val
                            self.vis.create_frustum_marker(frustum_marker, view, view.get_ptu_pose(), view_values)
                        idx += 1
                self.vis.pubfrustum.publish(frustum_marker)
                #vis.delete(p)
                break
            else:
                p = None
                
        if p.ID != best_plan_id:
            rospy.logerr("Something bad has happend!")

        userdata.views = p.views
        # visulaize for debug
        robot_poses  = PoseArray()
        robot_poses.header.frame_id = '/map'
        robot_poses.poses = []
        for v in views:
            robot_poses.poses.append(v.get_robot_pose())
            #print len(robot_poses.poses)
        self.robot_poses_pub.publish(robot_poses)
        
        return 'succeeded'



def trapezoidal_shaped_func(a, b, c, d, x):
    min_val = min(min((x - a)/(b - a), float(1.0)), (d - x)/(d - c))
    return max(min_val, float(0.0))


def r_func(x):
    a = -0.125
    b =  0.125
    c =  0.375
    d =  0.625
    x = 1.0 - x
    value = trapezoidal_shaped_func(a,b,c,d,x)
    return value

def g_func(x):
    a =  0.125
    b =  0.375
    c =  0.625
    d =  0.875
    x = 1.0 - x
    value = trapezoidal_shaped_func(a,b,c,d,x)
    return value


def b_func(x):
    a =  0.375
    b =  0.625
    c =  0.875
    d =  1.125
    x = 1.0 - x
    value = trapezoidal_shaped_func(a,b,c,d,x)
    return value



class Vis(object):
    def __init__(self):
        self._server = InteractiveMarkerServer("evaluated_plans")
        self.pubfrustum = rospy.Publisher('frustums', MarkerArray, queue_size=100)
        self.pubvoxel = rospy.Publisher('voxels', MarkerArray, queue_size=100)
        self.marker_id = 0
        self.voxel_id = 0

        
    def _update_cb(self,feedback):
        return

    def visualize_plan(self, plan, plan_values):
        int_marker = self.create_plan_marker(plan, plan_values)
        self._server.insert(int_marker, self._update_cb)
        self._server.applyChanges()

    
    def delete(self, plan):
        self._server.erase(plan.ID)
        self._server.applyChanges()

    def create_voxel_marker(self, markerArray, pose):
        marker1 = Marker()
        marker1.id = self.voxel_id
        self.voxel_id += 1
        marker1.header.frame_id = "/map"
        marker1.type = marker1.CUBE
        marker1.action = marker1.ADD
        marker1.scale.x = 0.05
        marker1.scale.y = 0.05
        marker1.scale.z = 0.05
        marker1.color.a = 1.0

        marker1.color.r = 0.0 
        marker1.color.g = 0.9
        marker1.color.b = 0.0
        marker1.pose.orientation = pose.orientation
        marker1.pose.position = pose.position

        markerArray.markers.append(marker1)
        

    def create_frustum_marker(self, markerArray, view, pose, view_values):
        marker1 = Marker()
        marker1.id = self.marker_id
        self.marker_id += 1
        marker1.header.frame_id = "/map"
        marker1.type = marker1.LINE_LIST
        marker1.action = marker1.ADD
        marker1.scale.x = 0.05
        marker1.color.a = 0.3

        vals = view_values.values()
        max_val = max(vals)
        non_zero_vals = filter(lambda a: a != 0, vals)
        min_val = min(non_zero_vals)
        
        print min_val, max_val, view_values[view.ID]
        
        marker1.color.r = r_func( float((view_values[view.ID] - min_val)) / float((max_val - min_val + 1)))
        marker1.color.g = g_func( float((view_values[view.ID] - min_val)) / float((max_val - min_val + 1)))
        marker1.color.b = b_func( float((view_values[view.ID] - min_val)) /  float((max_val - min_val + 1)))

        marker1.pose.orientation = pose.orientation
        marker1.pose.position = pose.position
 
        points = view.get_frustum()

        marker1.points.append(points[0])
        marker1.points.append(points[1])

        marker1.points.append(points[2])
        marker1.points.append(points[3])
        
        marker1.points.append(points[0])
        marker1.points.append(points[2])

        marker1.points.append(points[1])
        marker1.points.append(points[3])

        marker1.points.append(points[4])
        marker1.points.append(points[5])
        
        marker1.points.append(points[6])
        marker1.points.append(points[7])

        marker1.points.append(points[4])
        marker1.points.append(points[6])
        
        marker1.points.append(points[5])
        marker1.points.append(points[7])

        marker1.points.append(points[0])
        marker1.points.append(points[4])

        marker1.points.append(points[2])
        marker1.points.append(points[6])

        marker1.points.append(points[1])
        marker1.points.append(points[5])

        marker1.points.append(points[3])
        marker1.points.append(points[7])
        
        markerArray.markers.append(marker1)

        
    def create_plan_marker(self, plan, plan_values):
        # create an interactive marker for our server
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "/map"
        int_marker.name = plan.ID
        int_marker.description = plan.ID
        pose = Pose()
        #pose.position.x = traj.pose[0]['position']['x']
        #pose.position.y = traj.pose[0]['position']['y']
        int_marker.pose = pose
        
        line_marker = Marker()
        line_marker.type = Marker.LINE_STRIP
        line_marker.scale.x = 0.1

        # random.seed(float(plan.ID))
        # val = random.random()
        # line_marker.color.r = r_func(val)
        # line_marker.color.g = g_func(val)
        # line_marker.color.b = b_func(val)
        # line_marker.color.a = 1.0

        line_marker.points = []
        for view in plan.views:
            x = view.get_robot_pose().position.x
            y = view.get_robot_pose().position.y
            z = 0.0 # float(plan.ID) / 10
            p = Point()
            p.x = x - int_marker.pose.position.x  
            p.y = y - int_marker.pose.position.y
            p.z = z - int_marker.pose.position.z
            line_marker.points.append(p)

            line_marker.colors = []
            for i, view in enumerate(plan.views):
                color = ColorRGBA()
                val = float(i) / len(plan.views)
                color.r = r_func(val)
                color.g = g_func(val)
                color.b = b_func(val)
                color.a = 1.0
                line_marker.colors.append(color)
                

        # create a control which will move the box
        # this control does not contain any markers,
        # which will cause RViz to insert two arrows
        control = InteractiveMarkerControl()
        control.markers.append(line_marker) 
        int_marker.controls.append(control)
        
        return int_marker
