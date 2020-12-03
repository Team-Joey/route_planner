#!/usr/bin/python

import 	rospy
import 	route_planner
import 	route_planner.rp

import 	sys
from 	geometry_msgs.msg 	import Point, PoseStamped
from 	tf.msg 			import tfMessage
from 	sensor_msgs.msg 	import LaserScan
from 	nav_msgs.msg 		import OccupancyGrid, Odometry
from 	geometry_msgs.msg 	import Twist


class RoutePlannerNode(object):
	def __init__(self, robotname, placeholder_shopping_list):
		rospy.loginfo("Node initialisation...") 				# Information is only logged after a node instance has been created

		# ----- Minimum change DELTA before publishing a pose
		self._PUBLISH_DELTA = rospy.get_param("publish_delta", 0.1)
		self._latest_scan = None
		self._last_published_pose = None

		# robot name is a unique identifier e.g. robot_0, robot_1 etc
		current_pose_topic = robotname + "/current_pose"
		base_scan_topic = robotname + "/base_scan"
		odom_topic = robotname + "/odom"
		cmd_vel = robotname + "/cmd_vel"
		
		self._pose_publisher = rospy.Publisher(current_pose_topic, PoseStamped, queue_size=1)	# Publishes the position of the robot for visualisaion
		self._tf_publisher = rospy.Publisher("/tf", tfMessage, queue_size=1)			# Publishes tf message for debugging
		self._cmd_vel = rospy.Publisher(cmd_vel, Twist, queue_size=100)				# Publishes tf message for debugging
		
		self._route_planner = route_planner.rp.RoutePlanner(self._cmd_vel, placeholder_shopping_list)

		# subscribe to the odom and laser topics for this robot
		rospy.Subscriber(base_scan_topic, LaserScan, self._route_planner._laser_callback)
		rospy.Subscriber(odom_topic, Odometry, self._route_planner._odometry_callback)

		# ----- Then set the occupancy grid map
		rospy.loginfo("Waiting for a map...")
    		try:
			ocuccupancy_map = rospy.wait_for_message("/map", OccupancyGrid, 20)
		except:
			rospy.logerr("Problem getting a map. Check that you have a map_server"
                 			" running: rosrun map_server map_server <mapname> " )
			sys.exit(1)
		
		rospy.loginfo("Map received. %d X %d, %f m/px." % (ocuccupancy_map.info.width, ocuccupancy_map.info.height, ocuccupancy_map.info.resolution))
		self._route_planner.set_map(ocuccupancy_map)
		s = Point()
		e = Point()
		s.x = 44
		s.y = -54
		e.x = 131
		e.y = -182
		print(self._route_planner.A_star(s,e))
		exit()




#-----------------------------------------------------------------------------------------------------------------------------------------------------------
if __name__ == '__main__':
	rospy.init_node('Joey', anonymous = True) 	# (anonymous = True) ensures the name is unique for each node 
	rospy.loginfo("Creating the node instance...")


	placeholder_shopping_list = []
	# get all published topics
	# find how many robots are active
	topics = rospy.get_published_topics()
	robotnum = 0
	for i in range(0, len(topics)):
		if ("odom" in topics[i][0]):
			robotnum+=1

	# create as many nodes as there are robots in the map
	# if only one robot, do not index the name
	for i in range(0, robotnum):
		if robotnum == 1:
			RoutePlannerNode("", placeholder_shopping_list)
		else: 
			RoutePlannerNode("robot_"+str(i), placeholder_shopping_list)

	if (robotnum > 0):
		task_incomplete = True
		#while task_incomplete:				# Node is active until the task has been completed
		rospy.spin()
		rospy.loginfo("Task has been completed.")
		print("End.")
	else:
		print("No robots added to the world file, or stage_ros is not running")

