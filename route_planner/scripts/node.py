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
    def __init__(self, robotname):
	rospy.loginfo("Node initialisation...") 				# Information is only logged after a node instance has been created

	# ----- Minimum change DELTA before publishing a pose
	self._PUBLISH_DELTA = rospy.get_param("publish_delta", 0.1)
	self._route_planner = route_planner.rp.RoutePlanner() 
	self._route_planner.dummy_function_sum(3, 4)				# Accessing a function in the script

	self._latest_scan = None
	self._last_published_pose = None

	# robot name is a unique identifier e.g. robot_0, robot_1 etc
	current_pose_topic = robotname + "/current_pose"
	
	self._pose_publisher = rospy.Publisher(current_pose_topic, PoseStamped, queue_size=1)	# Publishes the position of the robot for visualisaion
	self._tf_publisher = rospy.Publisher("/tf", tfMessage, queue_size=1)			# Publishes tf message for debugging

	# ----- Then set the occupancy grid map
	""" 
	TO DO: 
	1. Uncomment the commented imports.
	2. set the occupancy grid map
	3. Define laser and odometry callback functions 
	4. Subscribe to /base_scan (lasers) and /odom (odometry)
	"""

#------------------------Following Functions Have NOT been Implemented-------------------------------------------------------------------------------------
    def _odometry_callback(self, odometry):
        """
	Function is called when a laser scan is received. 
	This function is activated by a subscriber to /odom
	in the kf node.py
        """
        raise NotImplementedError()

    def _laser_callback(self, scan):
        """
	Function is called when a laser scan is received. 
	This function is activated by a subscriber to /base_scan
	in the kf node.py
        """
        raise NotImplementedError()

#-----------------------------------------------------------------------------------------------------------------------------------------------------------
if __name__ == '__main__':
    rospy.init_node('dummy_node', anonymous = True) 	# (anonymous = True) ensures the name is unique for each node 
    rospy.loginfo("Creating the node instance...")

    # get all published topics
    # find how many robots are active
    # create that many nodes
    topics = rospy.get_published_topics()
    robotnum = 0
    for i in range(0, len(topics)):
    	if ("odom" in topics[i][0]):
    		RoutePlannerNode("robot_"+str(robotnum)) # Node initialisation
    		robotnum+=1

    if (robotnum > 0):
    	task_incomplete = True
    	#while task_incomplete:				# Node is active until the task has been completed
    	rospy.spin()
    	rospy.loginfo("Task has been completed.")
    	print("End.")
    else:
    	print("No robots found")