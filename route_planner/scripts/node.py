#!/usr/bin/python

import 	rospy
import 	route_planner
import 	route_planner.rp

import 	sys
from 	geometry_msgs.msg 	import PoseStamped
from 	tf.msg 			import tfMessage
# from 	sensor_msgs.msg 	import LaserScan
# from 	nav_msgs.msg 		import OccupancyGrid, Odometry


class RoutePlannerNode(object):
    def __init__(self):
	rospy.loginfo("Node initialisation...") 				# Information is only logged after a node instance has been created

	# ----- Minimum change DELTA before publishing a pose
	self._PUBLISH_DELTA = rospy.get_param("publish_delta", 0.1)
	self._route_planner = route_planner.rp.RoutePlanner() 
	self._route_planner.dummy_function_sum(3, 4)				# Accessing a function in the script

	self._latest_scan = None
        self._last_published_pose = None
	
	self._pose_publisher = rospy.Publisher("/current_pose", PoseStamped, queue_size=1)	# Publishes the position of the robot for visualisaion
        self._tf_publisher = rospy.Publisher("/tf", tfMessage, queue_size=1)			# Publishes tf message for debugging

	# ----- Then set the occupancy grid map

#-----------------------------------------------------------------------------------------------------------------------------------------------------------
if __name__ == '__main__':
    rospy.init_node('dummy_node', anonymous = True) 	# (anonymous = True) ensures the name is unique for each node 
    rospy.loginfo("Creating the node instance...")
    node = RoutePlannerNode() 				# Node initialisation
    
    task_incomplete = True
    while task_incomplete:				# Node is active until the task has been completed
	rospy.spin()
    rospy.loginfo("Task has been completed.")
    print("End.")
