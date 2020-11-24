import 	rospy
from 	geometry_msgs.msg 	import PoseStamped, Quaternion
from 	nav_msgs.msg 		import OccupancyGrid
from 	util 			import rotateQuaternion, getHeading
from 	tf.msg 			import tfMessage
from 	geometry_msgs.msg 	import Twist
import 	math
from 	math 			import cos, sin
import 	route_planner.movement


class RoutePlanner(object):

	#START_X = 0
	#START_Y = 0
	#START_Z = 0
	#START_HEADING = 0

	def __init__(self, _cmd_vel, shopping_list):
		rospy.loginfo("A route planner object was created.")
		self.current_pose = PoseStamped()			# Current pose of the robot
		self.occupancy_map = OccupancyGrid()			# Occupancy Grid map
		self.tf_message = tfMessage()				# tf message for debugging

	# ---- Setting the current pose based on the given starting point 
	#NOTE- this stuff is not needed, because the odom call back deals with this, commented for now
		#self.current_pose.pose.position.x = self.START_X
		#self.current_pose.pose.position.y = self.START_Y
		#self.current_pose.pose.position.z = self.START_Z
		#self.current_pose.pose.orientation = rotateQuaternion(Quaternion(w=1.0),
																	#self.START_HEADING)
		self.current_pose.header.frame_id = "/map"

		self.path_to_next_item = [[1,-1], [2,-1], [2,0]]
		self.shopping_list = shopping_list

		self._cmd_vel = _cmd_vel
		self.test = False

		# how close do two rotations need to get (in radians) before they are considered equal
		self.rotation_close_enough_threshold = 0.001
		# if the difference between two angles is less than this threshold, robot will slow down rot speed
		self.rotation_slow_threshold = 0.5
		self.angularSpeed = 10

		# if the distance to next position in the path is less than this value, robot will slow down
		self.movement_slow_threshold = 0.25
		self.linearSpeed = 10

		# create a movement object which will handle all translation and rotation of the robot
		self.movement = route_planner.movement.Movement(_cmd_vel)

#--------------------------------Dummy-------------------------------------------------------------------------------------------------------------------

	def dummy_function_sum(self, a, b):
		"""
		Dummy function. Sums 2 numbers.
		"""
		rospy.loginfo("Dummy function started")

		print("The sum of ", a, " and ", b," is ", a+b)

#------------------------Following Functions are currently being implemented-------------------------------------------------------------------------------------

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

#------------------------Following Functions Have NOT been Implemented-------------------------------------------------------------------------------------
	def _laser_callback(self, scan):
		x = 0

	def set_map(self, occupancy_map):
		"""Set the map"""
		raise NotImplementedError()

	def A_star(self, start_position, target_position):
		""" 
	Find shortest path from a given start position to 
	a given target position (using A* algorithm)
	Return: Array of coordinates the robot would visit
	"""
		raise NotImplementedError()

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

