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
import	math

ROBOTS = []
FOOD_ITEMS = []
MARKER_PUB = None
MAP_GRID = None

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
		
		self._route_planner = route_planner.rp.RoutePlanner(self._cmd_vel, placeholder_shopping_list, MAP_GRID)

		# subscribe to the odom and laser topics for this robot
		rospy.Subscriber(base_scan_topic, LaserScan, self._route_planner._laser_callback)
		rospy.Subscriber(odom_topic, Odometry, self._route_planner._odometry_callback)

# called every x times a second
def update():
	# array of markers to diplay in rviz, includes robots and food items
	markers = []

	# MAP_GRID probably needs to be updated here
	# MAP_GRID = update_map_grid()

	id = 0
	for robot in ROBOTS:
		# potentially should only call this function if the map has actually changed
		robot._route_planner.receive_map_update(MAP_GRID)

		# create a marker for the robot and append to marker array
		markers += (create_robot_marker(robot, id))

		# +2 because two markers are created- shape and text
		id += 2

	for food_item in FOOD_ITEMS:
		markers += (create_food_marker(food_item, id))

		# +2 because two markers are created- shape and text
		id += 2

	MARKER_PUB.publish(markers)

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
	robotMarker.pose.position.x = robot._route_planner.current_pose.pose.position.x + MAP_GRID.origin_x
	robotMarker.pose.position.y = robot._route_planner.current_pose.pose.position.y + MAP_GRID.origin_y
	robotMarker.pose.orientation = robot._route_planner.current_pose.pose.orientation

	robotMarker.lifetime = rospy.Duration(0)
	
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
	robotMarkerText.text=robot.name

	robotMarkerText.pose.position.x = robot._route_planner.current_pose.pose.position.x + MAP_GRID.origin_x
	robotMarkerText.pose.position.y = robot._route_planner.current_pose.pose.position.y + MAP_GRID.origin_y
	# give z of 2 so the text is above other markers
	robotMarkerText.pose.position.z = 2

	robotMarkerText.lifetime = rospy.Duration(0)
	
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

	# need to add offset to marker position
	foodMarker.pose.position.x = food_item.x
	foodMarker.pose.position.y = food_item.y

	foodMarker.lifetime = rospy.Duration(0)
	
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

	foodMarkerText.pose.position.x = food_item.x
	foodMarkerText.pose.position.y = food_item.y
	# give z of 2 so the text is above other markers
	foodMarkerText.pose.position.z = 2

	foodMarkerText.lifetime = rospy.Duration(0)
	
	foodMarkerText.scale.x = 0.5
	foodMarkerText.scale.y = 0.5
	foodMarkerText.scale.z = 0.5

	foodMarkerText.color.r = 1.0
	foodMarkerText.color.g = 0.0
	foodMarkerText.color.b = 1.0
	foodMarkerText.color.a = 1.0

	return [foodMarker, foodMarkerText]

def rotate(origin, point, angle):
    """
    Rotate a point counterclockwise by a given angle around a given origin.

    The angle should be given in radians.
    """
    ox, oy = origin
    px, py = point

    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    return qx, qy

def place_food():
	"""
	Place food around the map. Placed near walls (to replicate being on a shelf)
	minDistance is min distance food can be from a wall or other food item
	maxDistance is max
	"""

	food_to_place = 10
	minDistance = 20
	maxDistance = 25

	while (food_to_place >= 0):
		# choose random wall from map
		randindex = random.randrange(0, len(MAP_GRID.walls))
		coords = MAP_GRID.walls[randindex]

		food_placed = False

		x = (coords[0]*MAP_GRID.resolution)
		y = (coords[1]*MAP_GRID.resolution)

		# then find nearby open space
		for space in MAP_GRID.openNodes:
			# check distance
			dist = math.sqrt( (coords[0] - space[0])**2 + (coords[1] - space[1])**2 )

			# if the current space is close to a wall, but not too close, use this space
			if (dist < maxDistance and dist > minDistance):

				valid = True
				# check that this space is not too close to another food item
				for food in FOOD_ITEMS:
					dist = math.sqrt( (food.x/MAP_GRID.resolution - space[0])**2 + (food.y/MAP_GRID.resolution - space[1])**2 )
					if (dist < minDistance):
						valid = False
						break

				if (valid):
					# finally, check the surrounding space is clear of walls
					finalCheck = check_surroundings(space[0], space[1], minDistance)

					if (finalCheck):
						food_placed = True
						x = (space[0]*MAP_GRID.resolution)
						y = (space[1]*MAP_GRID.resolution)
						break

		# it's possible that no space was available to place the food, in this case, don't add anything
		if (food_placed):
			# coordinates need to be rotated by 90 degrees
			x,y = rotate((MAP_GRID.origin_x,MAP_GRID.origin_y),(x,y), -math.pi/2)

			# add food item to global list
			f = route_planner.food_item.FoodItem(x,y,"food")
			FOOD_ITEMS.append(f)

			# update how many food items are left to place
			food_to_place-=1

def check_surroundings(origin_x, origin_y, area_size):
	"""
	Given a pair of coordinates, check the surrounding area in the occupany map.
	Return True if all surrounding space is empty
	"""
	for x in range (-area_size, area_size):
		newX = x + origin_x
		if (newX > 0 and newX < len(MAP_GRID.OC_GRID_TEMP[0])):
			for y in range (-area_size, area_size):

				newY = y + origin_y

				if (newY > 0 and newY < len(MAP_GRID.OC_GRID_TEMP[1])):
					value = MAP_GRID.OC_GRID_TEMP[newX, newY]
					if not (value == 0):
						return False
	return True

def set_map(occupancy_map):
	"""Set the map"""
	MAP_GRID.set_map(occupancy_map)

#-----------------------------------------------------------------------------------------------------------------------------------------------------------
if __name__ == '__main__':
	print(math.sqrt(9))
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
	set_map(ocuccupancy_map)
	place_food()


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
			ROBOTS.append(RoutePlannerNode("", placeholder_shopping_list))
		else: 
			ROBOTS.append(RoutePlannerNode("robot_"+str(i), placeholder_shopping_list))

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

