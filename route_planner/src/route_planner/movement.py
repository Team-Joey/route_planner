import 	rospy
from 	geometry_msgs.msg 	import PoseStamped, Quaternion
from 	nav_msgs.msg 		import OccupancyGrid
from 	util 			import rotateQuaternion, getHeading
from 	tf.msg 			import tfMessage
from 	geometry_msgs.msg 	import Twist
import 	math
from 	math 			import cos, sin


class Movement(object):
	def __init__(self, _cmd_vel):
		self._cmd_vel = _cmd_vel

		# how close do two rotations need to get (in radians) before they are considered equal
		self.rotation_close_enough_threshold = 0.001
		# if the difference between two angles is less than this threshold, robot will slow down rot speed
		self.rotation_slow_threshold = 0.5
		self.angularSpeed = 10

		# if the distance to next position in the path is less than this value, robot will slow down
		self.distance_close_enough_threshold = 0.05
		self.movement_slow_threshold = 0.25
		self.linearSpeed = 10

#------------------------Following Functions are currently being implemented-------------------------------------------------------------------------------------

	def movement_update(self, current_target, odometry):
		"""
		Move/rotate the robot towards the given target
		"""
		self.current_pose = odometry.pose
		if self.rotate_to_target(current_target[0], current_target[1]):	
			return self.move_to_target(current_target[0], current_target[1])


#----------------------------------------------------------------------------------------------------------------------------------
	def move_to_target(self, x, y):
		"""
		Moves robot forward until the given position has been reached
		"""

		if (self.close_enough(x,y)):
			return True

		base_data = Twist()

		# get distance between current position and target
		distance = math.sqrt( (x - self.current_pose.pose.position.x)**2 + (y - self.current_pose.pose.position.y)**2 )

		speedmodifier = 1

		if (distance < self.movement_slow_threshold):
			speedmodifier = speedmodifier * distance 

		base_data.linear.x = self.linearSpeed * speedmodifier
		self._cmd_vel.publish( base_data )

		return False

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

		Returns true if the angle has been achieved.
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