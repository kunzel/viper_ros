#!/usr/bin/env python

import rospy
from strands_executive_msgs import task_utils
from strands_executive_msgs.msg import Task
from strands_executive_msgs.srv import DemandTask, SetExecutionStatus
# import strands_executive_msgs
import sys

def get_services():
    # get services necessary to do the jon
    demand_task_srv_name = '/task_executor/demand_task'
    set_exe_stat_srv_name = '/task_executor/set_execution_status'
    rospy.loginfo("Waiting for task_executor service...")
    rospy.wait_for_service(demand_task_srv_name)
    rospy.wait_for_service(set_exe_stat_srv_name)
    rospy.loginfo("Done")        
    add_tasks_srv = rospy.ServiceProxy(demand_task_srv_name, DemandTask)
    set_execution_status = rospy.ServiceProxy(set_exe_stat_srv_name, SetExecutionStatus)
    return add_tasks_srv, set_execution_status


if __name__ == '__main__':
    rospy.init_node("demand_object_search_task")

    if len(sys.argv)==5:
        waypoint=sys.argv[1]
        roi=sys.argv[2]
        surface_roi=sys.argv[3]
        mode=sys.argv[4]
        print "Using waypoint: ", waypoint
        print "Using ROI: ", roi
    else:
        print "Usage: demand_object_search_task.py waypoint roi_id surface_ros_id mode"
        sys.exit(1)

    # get services to call into execution framework
    demand_task, set_execution_status = get_services()

    # Set the task executor running (if it isn't already)
    set_execution_status(True)
    print 'set execution'

    task= Task(action='search_object', max_duration=rospy.Duration(600), start_node_id=waypoint)
    task_utils.add_string_argument(task, waypoint)
    task_utils.add_string_argument(task, roi)
    task_utils.add_string_argument(task, surface_roi)
    task_utils.add_string_argument(task, mode)
    resp = demand_task(task)
    print 'demanded task as id: %s' % resp.task_id
    rospy.loginfo('Success: %s' % resp.success)
    rospy.loginfo('Wait: %s' % resp.remaining_execution_time)

