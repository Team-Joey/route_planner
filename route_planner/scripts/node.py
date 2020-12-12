#!/usr/bin/python

import 	rospy
import 	route_planner
import 	route_planner.rp

import 	sys
from 	geometry_msgs.msg 	import PoseStamped
from 	tf.msg 			import tfMessage
from 	sensor_msgs.msg 	import LaserScan
from 	nav_msgs.msg 		import OccupancyGrid, Odometry
from 	geometry_msgs.msg 	import Twist
from 	visualization_msgs.msg import Marker
from 	visualization_msgs.msg import MarkerArray
import 	route_planner.map_grid
import 	route_planner.food_item
import 	random
import math
import copy

ROBOTS = []
FOOD_ITEMS = []
MARKER_PUB = None
MAP_GRID = None
MARKERS = []

# how long do markers stay in rviz before getting removed
# lower durations tend to lead to flashing markers
# and duration of zero leads to markers not deleting properly
MARKER_DURATION = 0.5

class RoutePlannerNode(object):
	def __init__(self, robotname, placeholder_shopping_list):
		rospy.loginfo("Node initialisation...") 				# Information is only logged after a node instance has been created

		# ----- Minimum change DELTA before publishing a pose
		self._PUBLISH_DELTA = rospy.get_param("publish_delta", 0.1)
		self._latest_scan = None
		self._last_published_pose = None
		self.name = robotname

		# robot name is a unique identifier e.g. robot_0, robot_1 etc
		current_pose_topic = robotname + "/current_pose"
		base_scan_topic = robotname + "/base_scan"
		odom_topic = robotname + "/odom"
		cmd_vel = robotname + "/cmd_vel"

		self._pose_publisher = rospy.Publisher(current_pose_topic, PoseStamped, queue_size=1)	# Publishes the position of the robot for visualisaion
		self._tf_publisher = rospy.Publisher("/tf", tfMessage, queue_size=1)			# Publishes tf message for debugging
		self._cmd_vel = rospy.Publisher(cmd_vel, Twist, queue_size=100)				# Publishes tf message for debugging

		self._route_planner = route_planner.rp.RoutePlanner(self._cmd_vel, placeholder_shopping_list, MAP_GRID, robotname)

		# subscribe to the odom and laser topics for this robot
		rospy.Subscriber(base_scan_topic, LaserScan, self._route_planner._laser_callback)
		rospy.Subscriber(odom_topic, Odometry, self._route_planner._odometry_callback)

def extract_robots():
	"""
	Collect all the matrix positions of each robot
	"""
	robots = []
	for robot in ROBOTS:
		robots.append(robot._route_planner)
	return robots


# called every x times a second
def update():
	# array of markers to diplay in rviz, includes robots and food items
	markers = []

	# extract RoutePlanner objects from RoutePlannerNodes (ROBOTS)
	robots = extract_robots()

	id = 0

	for robot in ROBOTS:
		# potentially should only call this function if the map has actually changed
		robot._route_planner.receive_map_update(robots)
		# create a marker for the robot and append to marker array
		markers += (create_robot_marker(robot, id))

		# +2 because two markers are created- shape and text
		id += 2

		count = 0
		for pos in robot._route_planner.path_to_next_item:

			x = pos[0]
			y = pos[1]

			markers.append(create_path_marker(x,y,id))#(create_food_marker(food_item, id))
			id += 1
			count+=1

		id+=1

	for food_item in FOOD_ITEMS:
		markers += (create_food_marker(food_item, id))

		# +2 because two markers are created- shape and text
		id += 2

	MARKER_PUB.publish(markers)

def create_path_marker(x,y,id):
	marker = Marker()
	marker.header.frame_id = "/map"
	marker.header.stamp    = rospy.get_rostime()
	#marker.ns = robot.name
	marker.id = id
	marker.type = 1 # sphere
	marker.action = 0

	# need to add offset to marker position
	real_x, real_y = MAP_GRID.matrix_to_real(x, y)
	marker.pose.position.x = real_x
	marker.pose.position.y = real_y

	marker.lifetime = rospy.Duration(MARKER_DURATION)

	marker.scale.x = 0.25
	marker.scale.y = 0.25
	marker.scale.z = 0.25

	marker.color.r = 1.0
	marker.color.g = 1.0
	marker.color.b = 0.0
	marker.color.a = 1.0

	return marker

# returns an array containing a shape marker and a text marker
def create_robot_marker(robot, id):
	robotMarker = Marker()
	robotMarker.header.frame_id = "/map"
	robotMarker.header.stamp    = rospy.get_rostime()
	robotMarker.ns = robot.name
	robotMarker.id = id
	robotMarker.type = 1 # sphere
	robotMarker.action = 0

	# need to add offset to marker position
	robotMarker.pose.position.x = robot._route_planner.current_pose.pose.position.x
	robotMarker.pose.position.y = robot._route_planner.current_pose.pose.position.y
	robotMarker.pose.orientation = robot._route_planner.current_pose.pose.orientation

	robotMarker.lifetime = rospy.Duration(MARKER_DURATION)

	robotMarker.scale.x = 0.5
	robotMarker.scale.y = 0.5
	robotMarker.scale.z = 0.5

	robotMarker.color.r = 0.0
	robotMarker.color.g = 1.0
	robotMarker.color.b = 0.0
	robotMarker.color.a = 1.0


	robotMarkerText = Marker()
	robotMarkerText.header.frame_id = "/map"
	robotMarkerText.header.stamp    = rospy.get_rostime()
	robotMarkerText.ns = robot.name + "_text"

	robotMarkerText.id = id + 1
	robotMarkerText.type = Marker.TEXT_VIEW_FACING#2 # sphere
	robotMarkerText.action = 0
	robotMarkerText.text=str(robot._route_planner.status)

	robotMarkerText.pose.position.x = robot._route_planner.current_pose.pose.position.x
	robotMarkerText.pose.position.y = robot._route_planner.current_pose.pose.position.y
	# give z of 2 so the text is above other markers
	robotMarkerText.pose.position.z = 2

	robotMarkerText.lifetime = rospy.Duration(MARKER_DURATION)

	robotMarkerText.scale.x = 0.5
	robotMarkerText.scale.y = 0.5
	robotMarkerText.scale.z = 0.5

	robotMarkerText.color.r = 0.0
	robotMarkerText.color.g = 0.0
	robotMarkerText.color.b = 1.0
	robotMarkerText.color.a = 1.0

	return [robotMarker, robotMarkerText]

# returns an array containing a shape marker and a text marker
def create_food_marker(food_item, id):
	foodMarker = Marker()
	foodMarker.header.frame_id = "/map"
	foodMarker.header.stamp    = rospy.get_rostime()
	foodMarker.ns = food_item.label
	foodMarker.id = id
	foodMarker.type = 2
	foodMarker.action = 0

	# convert to real-world position so marker is displayed in correct place
	real_x, real_y = MAP_GRID.matrix_to_real(food_item.y, food_item.x)
	foodMarker.pose.position.x = real_x
	foodMarker.pose.position.y = real_y

	foodMarker.lifetime = rospy.Duration(MARKER_DURATION)

	foodMarker.scale.x = 0.5
	foodMarker.scale.y = 0.5
	foodMarker.scale.z = 0.5

	foodMarker.color.r = 1.0
	foodMarker.color.g = 0.0
	foodMarker.color.b = 1.0
	foodMarker.color.a = 1.0


	foodMarkerText = Marker()
	foodMarkerText.header.frame_id = "/map"
	foodMarkerText.header.stamp    = rospy.get_rostime()
	foodMarkerText.ns = food_item.label + "_text"

	foodMarkerText.id = id + 1
	foodMarkerText.type = Marker.TEXT_VIEW_FACING
	foodMarkerText.action = 0
	foodMarkerText.text=food_item.label

	foodMarkerText.pose.position.x = real_x
	foodMarkerText.pose.position.y = real_y
	# give z of 2 so the text is above other markers
	foodMarkerText.pose.position.z = 2

	foodMarkerText.lifetime = rospy.Duration(MARKER_DURATION)

	foodMarkerText.scale.x = 0.5
	foodMarkerText.scale.y = 0.5
	foodMarkerText.scale.z = 0.5

	foodMarkerText.color.r = 1.0
	foodMarkerText.color.g = 0.0
	foodMarkerText.color.b = 1.0
	foodMarkerText.color.a = 1.0

	return [foodMarker, foodMarkerText]

def sort(products_array, current_position):
	""" Naive approach to sorting"""
	print("Sorting started")
	" If the list of products contains 1 or 0 items, the sorted list would be the same as the list of items"
	# Base cases
	if((len(products_array) == 0) or (len(products_array) == 1)):
		return products_array

	sorted_array = []
	shortest_path = len(A_star(current_position, products_array[0]))
	shortest_index = 0
	closest_product = products_array[0]

	# Find the shortest path to next item. If 2 items are equaly close,
	# Take the one with the smaller index value
	for product in products_array:
		if (len(A_star(current_position, product)) < shortest_path):
			shortest_path = len(A_star(current_position, product))
			shortest_index = products_array.index(product)
			closest_product = product

	# Add the closest product to the sorted array
	sorted_array.append(closest_product)
	products_array.pop(shortest_index)

	# Recursively sort the entire list
	return sorted_array + (sort(products_array, closest_product))

def place_food():
	"""
	Place food around the map. Placed near walls (to replicate being on a shelf)
	minDistance is min distance food can be from a wall or other food item
	maxDistance is max
	"""
	print("Placing food")
	food_to_place = 20
	minDistance = 15
	maxDistance = 20

	spaces = []

	# repeat process for open spaces
	for space in MAP_GRID.openNodes:

		# list of walls has not been resolution-reduced, so have to do that here
		x = space[1] / MAP_GRID.resolution_reduction_scale
		y = space[0] / MAP_GRID.resolution_reduction_scale

		spaces.append([x,y])

	# because a while loop is used, having an iteration limit to prevent freezing if no spaces can be found for food
	allowed_its = food_to_place * 100
	while (food_to_place > 0 and allowed_its > 0):

		# choose random wall from map
		randindex = random.randrange(0, len(spaces))
		coords = spaces[randindex]

		if check_surroundings(coords[1], coords[0], minDistance):
			# check that this space is not too close to another food item
			valid = True
			for food in FOOD_ITEMS:
				dist = math.sqrt( (food.x - coords[0])**2 + (food.y - coords[1])**2 )
				if (dist < minDistance):
					valid = False
					break
			if (valid):
				# get ascii code and convert to char
				label = chr(65 + food_to_place)
				f = route_planner.food_item.FoodItem(coords[0],coords[1], label)
				FOOD_ITEMS.append(f)
				food_to_place-=1

		allowed_its -= 1

def check_surroundings(origin_x, origin_y, area_size):
	"""
	Given a pair of coordinates, check the surrounding area in the occupany map.
	Return True if all surrounding space is empty
	"""

	x = int(origin_x * MAP_GRID.resolution_reduction_scale)
	y = int(origin_y * MAP_GRID.resolution_reduction_scale)

	# lower areas sizes don't work so well, so increase here
	area_size = int(area_size*3)

	for x in range (-area_size, area_size):
		newX = x + origin_x
		if (newX > 0 and newX < len(MAP_GRID.OC_GRID_TEMP[0])):
			for y in range (-area_size, area_size):

				newY = y + origin_y

				if (newY > 0 and newY < len(MAP_GRID.OC_GRID_TEMP[1])):
					value = MAP_GRID.OC_GRID_TEMP[int(newX), int(newY)]
					if not (value == 0):
						return False
	return True

def create_shopping_list():
	"""
	Return a random list of items from the global FOOD_ITEMS list
	"""

	shopping_list = []

	# min and max number of items allowed in shopping list
	# assumes FOOD_ITEMS is as least long as max
	min = 1
	max = 2

	count = random.randrange(min, max)

	for i in range(0, count):
		randindex = random.randrange(0, len(FOOD_ITEMS))
		food = FOOD_ITEMS[randindex]
		shopping_list.append(food)

	return shopping_list

#-----------------------------------------------------------------------------------------------------------------------------------------------------------
if __name__ == '__main__':
	rospy.init_node('Joey', anonymous = True) 	# (anonymous = True) ensures the name is unique for each node
	rospy.loginfo("Creating the node instance...")

	# ----- Then set the occupancy grid map
	rospy.loginfo("Waiting for a map...")
	try:
		ocuccupancy_map = rospy.wait_for_message("/map", OccupancyGrid, 20)
	except:
		rospy.logerr("Problem getting a map. Check that you have a map_server"
						" running: rosrun map_server map_server <mapname> " )
		sys.exit(1)

	rospy.loginfo("Map received. %d X %d, %f m/px." % (ocuccupancy_map.info.width, ocuccupancy_map.info.height, ocuccupancy_map.info.resolution))
	MAP_GRID =  route_planner.map_grid.MapGrid()
	MAP_GRID.set_map(ocuccupancy_map)
	place_food()

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
		shopping_list = create_shopping_list()
		if robotnum == 1:
			ROBOTS.append(RoutePlannerNode("", shopping_list))
		else:
			ROBOTS.append(RoutePlannerNode("robot_"+str(i),shopping_list))

	if (robotnum > 0):
		task_incomplete = True

		MARKER_PUB = rospy.Publisher('markers', MarkerArray, queue_size=10)

		while (task_incomplete):
			update()
			rospy.sleep(0.01)

		rospy.loginfo("Task has been completed.")
		print("End.")
	else:
		print("No robots added to the world file, or stage_ros is not running")
