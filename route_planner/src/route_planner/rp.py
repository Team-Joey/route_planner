import 	rospy
from 	geometry_msgs.msg 	import PoseStamped, Quaternion
from 	nav_msgs.msg 		import OccupancyGrid
from 	util 			import rotateQuaternion, getHeading
from 	tf.msg 			import tfMessage
from 	geometry_msgs.msg 	import Twist
import 	math
from 	math 			import cos, sin
import 	route_planner.movement

class N:
	def __init__(self, p):
		self.parent = None
		self.p = p
		self.isObstacle = false
		self.visited = false
		self.gDist = 0.0
		self.lDist = 0.0
		self.neighbours = []

class RoutePlanner(object):

	def __init__(self, _cmd_vel, shopping_list):
		rospy.loginfo("A route planner object was created.")
		self.current_pose = PoseStamped()			# Current pose of the robot
		self.occupancy_map = OccupancyGrid()			# Occupancy Grid map
		self.tf_message = tfMessage()				# tf message for debugging
		
		self.current_pose.header.frame_id = "/map"

		self.path_to_next_item = [[1,-1], [2,-1], [2,0]]
		self.shopping_list = shopping_list

		self._cmd_vel = _cmd_vel

		# create a movement object set_mapwhich will handle all translation and rotation of the robot
		self.movement = route_planner.movement.Movement(_cmd_vel)

		self.current_pose.header.frame_id = "/map"

#------------------------Following Functions are currently being implemented-------------------------------------------------------------------------------------

	def receive_map_update(self, map_grid):
		# do something
		x = 0


	def _odometry_callback(self, odometry):
		"""
		The function is called when the robot moves.
		If the robot's task list is not empty, the function
		checks if it is close enough. then updates target.
		"""
		self.current_pose = odometry.pose

		if len(self.path_to_next_item) == 0: 
			print("Finished path, either we are done or need to get path to next item")

		else:

			current_target = self.path_to_next_item[0]

			# call the movement function, returns true if robot has reached target
			if (self.movement.movement_update(current_target, odometry)):

				self.path_to_next_item.remove(current_target)

				if len(self.path_to_next_item) > 0:
					# ---- Change target and remove it from task list
					current_target = self.path_to_next_item[0]

# ----------------------------------------------------------------------------

	def distance(self, p1, p2):
        	return sqrt( (p2.x - p1.x)**2 + (p2.y - p1.y)**2 )

	def heuristic(self, p1, p2):
	        return distance(p1, p2)

    	def A_star(self, start_position, target_position): 
	    	"""
		Find shortest path from a given start position to 
	  	a given target position (using A* algorithm)
		Return: Array of coordinates the robot would visit
		"""
        	self.startN = None
        	self.endN = None
		
		for n in range(0, len(gridNodes)):
		    gridNodes(n).visited = false
		    gridNodes(n).gDist = 99999999
		    gridNodes(n).lDist = 99999999
		    gridNodes(n).parent = None
		    if (gridNodes(n).p == start_position): #add 3x3 square offset
		        self.startN = gridNodes(n)
		    if (gridNodes(n).p == target_position): #add 3x3 square offset
		        self.endN = gridNodes(n)
		
		self.startN.lDist = 0.0
		self.startN.gDist = heuristic(start_position, target_position)
		self.currentN = self.startN

		self.notTestedNodes = []
		self.notTestedNodes.append(self.startN)
		
		""" and (self.currentN.p != target_position) """
		while ((not self.notTestedNodes)):
		    
		    for n in range(0, len(self.notTestedNodes)):
		        if (self.notTestedNodes(n).visited):
		            self.notTestedNodes.remove(self.notTestedNodes(n))

		    if (not self.notTestedNodes):
		        break

		    for n in range(0, len(self.notTestedNodes)):
		        if (self.currentN.gDist > self.notTestedNodes(n).gDist):
		            self.currentN = self.notTestedNodes(n)
		    
		    self.currentN.visited = true

		    for n in range(0, len(self.currentN.neighbours)):

		        if ((not self.currentN.neighbours(n).visited) and (not self.currentN.neighbours(n).isObstacle)):
		            self.notTestedNodes.append(self.currentN.neighbours(n))

		        self.possiblyLowerDist = self.currentN.lDist + distance(self.currentN.p, self.currentN.neighbours(n).p)

		        if (self.possiblyLowerDist < self.currentN.neighbours(n).lDist):
		            self.currentN.neighbours(n).parent = self.currentN
		            self.currentN.neighbours(n).lDist = self.possiblyLowerDist
		            self.currentN.neighbours(n).gDist = self.currentN.neighbours(n).lDist + heuristic(self.currentN.neighbours(n).p, target_position)

#------------------------Following Functions Have NOT been Implemented-------------------------------------------------------------------------------------
	def _laser_callback(self, scan):
		x = 0


	def find_coordinate(self, product):			# Might need more arguments
		""" 
	Find the coordinate of the given product
	using the map
	Return: coordinates of the product
	"""
		raise NotImplementedError()

	def sort(self, products_array):
		"""
	This function sorts the array of products in some order
	Returns: sorted products_array
	"""
		raise NotImplementedError()

