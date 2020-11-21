import 	rospy
from 	geometry_msgs.msg 	import PoseStamped, Quaternion
from 	nav_msgs.msg 		import OccupancyGrid
from 	util 			import rotateQuaternion, getHeading
from 	tf.msg 			import tfMessage


class RoutePlanner(object):
    START_X = 0
    START_Y = 0
    START_Z = 0
    START_HEADING = 0

    def __init__(self):
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

#--------------------------------Dummy-------------------------------------------------------------------------------------------------------------------

    def dummy_function_sum(self, a, b):
	"""
	Dummy function. Sums 2 numbers.
	"""
	rospy.loginfo("Dummy function started")

	print("The sum of ", a, " and ", b," is ", a+b)

#------------------------Following Functions Have NOT been Implemented-------------------------------------------------------------------------------------

    def _odometry_callback(self, odometry):
    	print("recieving odom")

    def _laser_callback(self, scan):
    	print("recieving scan")

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

