import 	rospy
from 	geometry_msgs.msg 	import PoseStamped, Quaternion
from 	nav_msgs.msg 		import OccupancyGrid
from 	util 			import rotateQuaternion, getHeading
from 	tf.msg 			import tfMessage
from 	geometry_msgs.msg 	import Twist
import 	math
from 	math 			import cos, sin
import 	route_planner.movement
import 	map_grid

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

        # create a movement object which will handle all translation and rotation of the robot
        self.movement = route_planner.movement.Movement(_cmd_vel)

        self.current_pose.header.frame_id = "/map"
        # ----- Sensor model
        self.map_grid =  map_grid.MapGrid()
        
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

# ----------------------------------------------------------------------------

    def distance(self, p1, p2):
        return math.sqrt( (p2.x - p1.x)**2 + (p2.y - p1.y)**2 )

    def heuristic(self, p1, p2):
        return self.distance(p1, p2)

    def A_star(self, start_position, target_position):
        """
        Find shortest path from a given start "position" to 
        a given target "position" (using A* algorithm)
        Return: Array of pixel "coordinates" the robot would visit
        """
        Nodes = self.map_grid.gridNodes
        startNodeIndex = None
        endNodeIndex = None

        #Reset all nodes' parent, visited and distance values
        for n in range(0, len(Nodes)):
            Nodes[n].visited = False
            Nodes[n].lDist = 999999999999999999
            Nodes[n].gDist = 999999999999999999
            Nodes[n].parent = None
            #Set starting and ending nodes
            if (Nodes[n].p == start_position): 
                startNodeIndex = n
            if (Nodes[n].p == target_position): 
                endNodeIndex = n
        
        print("start:", startNodeIndex)
        print("end:", endNodeIndex)

        Nodes[startNodeIndex].lDist = 0.0
        Nodes[startNodeIndex].gDist = self.heuristic(start_position, target_position)
        print(Nodes[startNodeIndex].gDist)
        cNI = startNodeIndex

        notTestedNI = []
        #Add start node to list of not visited nodes
        notTestedNI.append(startNodeIndex)
        
        #Loop while there are nodes to test
        while (notTestedNI):
            
            #Set current node to node with the least g distance
            cNI = notTestedNI[0]
            for n in range(0, len(notTestedNI)):
                if (Nodes[cNI].gDist > Nodes[notTestedNI[n]].gDist):
                    cNI = notTestedNI[n]
            
            #print(cNI)
            #print(Nodes[cNI].neighbours)
            Nodes[cNI].visited = True
            #print(Nodes[cNI].visited)
            #End loop if path is found
            if cNI == endNodeIndex:
                break
            
            #Loop through the current nodes' neighbours or "children"
            for n in range(0, len(Nodes[cNI].neighbours)):
                
                #Add neighbour to list if it hasn't been visited and isn't an obstacle
                k = Nodes[cNI].neighbours[n]
                if ((Nodes[k].visited == False) and (Nodes[k].isObstacle == False)):
                    if k not in notTestedNI: notTestedNI.append(k)

                possiblyLowerDist = Nodes[cNI].lDist + self.distance(Nodes[cNI].p, Nodes[k].p)

                if (possiblyLowerDist < Nodes[k].lDist):
                    Nodes[k].parent = cNI
                    Nodes[k].lDist = possiblyLowerDist
                    Nodes[k].gDist = Nodes[k].lDist + self.heuristic(Nodes[k].p, target_position)

            #Remove visited node
            notTestedNI.remove(cNI)
            #print(notTestedNI)            
        
        path = []
        if cNI == endNodeIndex:
            while Nodes[cNI].parent != None:
                x = Nodes[cNI].p.x
                y = -Nodes[cNI].p.y
                path.append([x,y])
                cNI = Nodes[cNI].parent
            path.reverse()
        return path

    def set_map(self, occupancy_map):
        """Set the map"""
        self.map_grid.set_map(occupancy_map)
        
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

