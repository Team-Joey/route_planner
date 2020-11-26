import 	rospy
from 	geometry_msgs.msg 	import PoseStamped, Quaternion
from 	nav_msgs.msg 		import OccupancyGrid
from 	util 			import rotateQuaternion, getHeading
from 	tf.msg 			import tfMessage

import map_grid

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
	    self.current_pose.pose.orientation = rotateQuaternion(Quaternion(w=1.0), self.START_HEADING)
	    self.current_pose.header.frame_id = "/map"
        # ----- Sensor model
	    self.map_grid =  map_grid.MapGrid()
        
        # ----- Create list of 3x3? pixels nodes from occupancy grid
    
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

        """
        openlist = []
        closedlist = []

        openlist.append(startN)

        while openlist != empty
            currentN = lowest N.f in openlist
            openlist.remove(currentN)
            closedlist.append(currentN)

            if currentN = goal
                done
            
            currentN.children = adjNs

            for child in children
                if child in closedlist
                    do nothing
                else
                    child.g = currentN.g + d between child & current
                    child.h = d from child to goal
                    child.f = child.g + child.h

                    if child.position is in openlist's nodes positions
                        if the child.g is higher than the openlist node's g
                            do nothing
                    else
                        openlist.append(child)
        """


    def set_map(self, occupancy_map):
        """Set the map"""
        self.map_grid.set_map(occupancy_map)


#--------------------------------Dummy-------------------------------------------------------------------------------------------------------------------

"""     def dummy_function_sum(self, a, b):
	"""
	#Dummy function. Sums 2 numbers.
	"""
	rospy.loginfo("Dummy function started")

	print("The sum of ", a, " and ", b," is ", a+b) """

#------------------------Following Functions Have NOT been Implemented-------------------------------------------------------------------------------------

    def _odometry_callback(self, odometry):
    	print("recieving odom")

    def _laser_callback(self, scan):
    	print("recieving scan")

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

