import 	rospy
from 	geometry_msgs.msg 	import PoseStamped, Quaternion
from 	nav_msgs.msg 		import OccupancyGrid
from 	util 			import rotateQuaternion, getHeading
from 	tf.msg 			import tfMessage
from 	geometry_msgs.msg 	import Twist
import 	math
from 	math 			import cos, sin


class RoutePlanner(object):
	START_X = 0
	START_Y = 0
	START_Z = 0
	START_HEADING = 0
	EPHSILON = 0.3

	def __init__(self, _cmd_vel):
		rospy.loginfo("A route planner object was created.")
		self.current_pose = PoseStamped()			# Current pose of the robot
		self.occupancy_map = OccupancyGrid()			# Occupancy Grid map
		self.tf_message = tfMessage()				# tf message for debugging

	# ---- Setting the current pose based on the given starting point 
		self.current_pose.pose.position.x = self.START_X
		self.current_pose.pose.position.y = self.START_Y
		self.current_pose.pose.position.z = self.START_Z
		self.current_pose.pose.orientation = rotateQuaternion(Quaternion(w=1.0),
																	self.START_HEADING)
		self.current_pose.header.frame_id = "/map"
		self.path_to_next_item = []

		self._cmd_vel = _cmd_vel
		self.test = False

		# how close do two rotations need to get (in radians) before they are considered equal
		self.rotation_close_enough_threshold = 0.001
		# if the difference between two angles is less than this threshold, robot will slow down rot speed
		self.rotation_slow_threshold = 0.5
		self.angularSpeed = 10

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
		current_target = [1, -1]
		base_data = Twist()
		self.current_pose = odometry.pose
		if self.test: 
			return
		if len(self.path_to_next_item) > 0: # change to == 0
			#wait
			print("Not waiting :P.")
		else:
			if(self.close_enough(current_target[0], current_target[1])):
				# ---- Remove target from array --> next element becomes the 1st
				self.path_to_next_item.remove(self.current_target)
				if len(self.path_to_next_item) > 0:
					# ---- Change target and remove it from task list
					self.current_target = self.path_to_next_item[0]
				else:	
					print("len is zero. (in odom_callback) ")
					return
					# Go to next shopping item
				
			else: 
				# ---- Move the robot forward (towards the target) 
				base_data.linear.x = 0.1
				if self.rotate_to_target(current_target[0], current_target[1]):	
					self.test = True

			#print("Received odometry.")

#----------------------------------------------------------------------------------------------------------------------------------
	def rotate_to_target(self, x, y):
		"""
		Rotates the robot to the direction of a given x and y.
		This function calculates the angle the robot needs to rotate by, and then
		calls a separate function to actually perform the rotation.
		"""
		# v1 is the target vector = target pos - current
		v1 = [x - self.current_pose.pose.position.x, y - self.current_pose.pose.position.y]

		robotangle = getHeading(self.current_pose.pose.orientation)

		# current vector will be current pos + 1 in whatever direction we are facing
		pos = self.project_vector(self.current_pose.pose.position.x, self.current_pose.pose.position.y, 1, robotangle)

		v2 = [pos[0] - self.current_pose.pose.position.x, pos[1] - self.current_pose.pose.position.y]

		# signed angle between v1 and v2
		signed_angle_rad = math.atan2( v1[0]*v2[1] - v1[1]*v2[0], v1[0]*v2[0] + v1[1]*v2[1]);

		return self.rotate_by_angle(signed_angle_rad, False) 
#----------------------------------------------------------------------------------------------------------------------------------
	def project_vector(self, x, y, distance, angle):
		"""
		Projecting a point through a distance and angle to get a vector.
		"""
		newx = distance*cos(angle) + x
		newy = distance*sin(angle) + y
		
		return newx,newy
#----------------------------------------------------------------------------------------------------------------------------------
	def rotate_by_angle(self, angle, waittilldone):
		"""
		Given an angle rotate the robot in the 
		direction of that angle. 
		If waittilldone is True rospy will wait for the rotation to be completed. 
		Else, the rotation might not be completed in this call.
		"""
		self.rotation = self.current_pose.pose.orientation
		
		# first turn given angle to quaternion:
		self.targetRot = getHeading(self.current_pose.pose.orientation) + angle

		self.currentRot = getHeading(self.current_pose.pose.orientation)

		direction = 1
		if (self.targetRot > self.currentRot):
			direction = -1

		while (True):
			speedmodifier = 1

			self.currentRot = getHeading(self.current_pose.pose.orientation)

			dif = abs(self.targetRot - self.currentRot) # absolute value of the difference between the target rotation and the current rotation

			# if difference is very small, this close enough to be considered the same angle
			if (dif < self.rotation_close_enough_threshold):
				return True

			# if difference is small, slow the speed of rotation
			if (dif < self.rotation_slow_threshold):
				speedmodifier = 2 * dif / self.angularSpeed

			base_data = Twist()

			base_data.angular.z = self.angularSpeed * direction * speedmodifier
			self._cmd_vel.publish( base_data )

			if(waittilldone):
				rospy.sleep(0.1)
			else:
				# if only running this once, break out
				break

		return False

#----------------------------------------------------------------------------------------------------------------------------------
	def close_enough(self, x, y):
		"""
		Checks if current possition is close enough to a target
		"""
		EPHSILON = 0.3
		if (abs(x - self.current_pose.pose.position.x) > EPHSILON):
			return False
		if (abs(y - self.current_pose.pose.position.y) > EPHSILON):
			return False
		return True

#------------------------Following Functions Have NOT been Implemented-------------------------------------------------------------------------------------
	def _laser_callback(self, scan):
		print(".")

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

