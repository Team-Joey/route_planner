import 	rospy
from 	geometry_msgs.msg 	import PoseStamped, Quaternion, Point
from 	nav_msgs.msg 		import OccupancyGrid
from 	util 			import rotateQuaternion, getHeading
from 	tf.msg 			import tfMessage
from 	geometry_msgs.msg 	import Twist
import 	math
from 	math 			import cos, sin
import 	route_planner.movement
import 	math
import 	random
import copy

class RoutePlanner(object):

	def __init__(self, _cmd_vel, shopping_list, map_grid, name):
		rospy.loginfo("A route planner object was created.")

		self.path_to_next_item = []
		self.current_pose = PoseStamped()			# Current pose of the robot
		self.occupancy_map = OccupancyGrid()			# Occupancy Grid map
		self.tf_message = tfMessage()				# tf message for debugging
		
		self.current_pose.header.frame_id = "/map"

		self.map_grid = copy.deepcopy(map_grid)

		self.path_to_next_item = []

		self.shopping_list = shopping_list

		self._cmd_vel = _cmd_vel

		# create a movement object set_mapwhich will handle all translation and rotation of the robot
		self.movement = route_planner.movement.Movement(_cmd_vel)

		self.wait_for_obstacle_to_move = False

		# text to display above robot in rviz, useful for letting us know what the robot is doing
		self.status = ""

		self.name = name

		# a list of robots that are blocking this robot's path
		self.blocked_by = None

		self.is_waiting = False

		self.blocked_nodes = []

#------------------------Following Functions are currently being implemented-------------------------------------------------------------------------------------

	def receive_map_update(self, robots):
		self.check_path_for_robot_obstacles(robots)



	def _odometry_callback(self, odometry):
		"""
		The function is called when the robot moves.
		If the robot's task list is not empty, the function
		checks if it is close enough. then updates target.
		"""
		self.current_pose = odometry.pose

		# before adding origin, odom needs to be scaled 
		scalefactor = 0.725

		self.current_pose.pose.position.x *= scalefactor
		self.current_pose.pose.position.y *= scalefactor

		self.current_pose.pose.position.x += self.map_grid.origin_x
		self.current_pose.pose.position.y += self.map_grid.origin_y

		#if self.name == "robot_1":
			#print((self.map_grid.real_to_matrix(self.current_pose.pose.position.x, self.current_pose.pose.position.y)))
		#	return

		# don't allow any action if waiting
		if self.is_waiting:
			return

		# if another robot is in the way, wait for them to move
		# issue order to stop movement
		if not self.blocked_by == None:
			self.wait_or_replan()
			return

		if len(self.path_to_next_item) == 0: 
				print("Finished path, either we are done or need to get path to next item")
				self.path_to_next_item = self.new_path()
		else:
			self.status = "Following path"
			# next position on the current path, matrix position
			current_target = self.path_to_next_item[0]

			real_x, real_y = self.map_grid.matrix_to_real(current_target[0], current_target[1])

			real_target = [real_x, real_y]

			# call the movement function, returns true if robot has reached target
			if (self.movement.movement_update(real_target, odometry, self.map_grid)):

				self.path_to_next_item.remove(current_target)

				if len(self.path_to_next_item) > 0:
					# ---- Change target and remove it from task list
					current_target = self.path_to_next_item[0]

	def new_path(self):

		self.blocked_nodes = []

		print("Getting path to random food item")

		# convert current position to matrix
		startx, starty = self.map_grid.real_to_matrix(self.current_pose.pose.position.x, self.current_pose.pose.position.y)

		s = Point(startx, -starty, 0)

		# choose random food item from shopping list as target
		randindex = random.randrange(0, len(self.shopping_list))
		f = self.shopping_list[randindex]

		# NOTE: converting to integer is needed otherwise path will not be found
		e = Point(int(f.x),int(-f.y),0)

		path = self.A_star(s,e, [])

		return self.trim_path(path)

	def trim_path(self, path):
		# cut out some waypoints, slows robot down a lot otherwise
		trimmed_path = []

		# ratio is how many waypoints will be skipped when trimming 
		ratio = 4

		count = 0
		for pos in path:
			if (count == ratio):
				count = 0
				trimmed_path.append(pos)
			count += 1

		return trimmed_path

	def check_path_for_robot_obstacles(self, robots):
		"""
		Sets the blocked_by list (fills it with any robots that are blocking this robot)
		"""

		self.blocked_by = None

		self.robot_detection_range = 1

		if (len(self.path_to_next_item) == 0):
			self.status = "No path"
			return

		# get next position in list and convert to real
		for r in robots:
			# don't apply this for this robot object
			if not(r == self):

				for i in range(0, 2):
					if (i < len(self.path_to_next_item)):
						self_x, self_y = self.map_grid.matrix_to_real(self.path_to_next_item[i][0], self.path_to_next_item[i][1])

						robot_x = r.current_pose.pose.position.x
						robot_y = r.current_pose.pose.position.y

						dist = math.sqrt( (robot_x - self_x)**2 + (robot_y - self_y)**2 )

						# if the other robot is too close, add to list of blocking robots
						if (dist < self.robot_detection_range):
							self.blocked_by = r
							return

		# if no robots are blocking this robot, make sure to uncheck waiting
		self.is_waiting = False

	def wait_or_replan(self):
		# first kill any movement
		self.movement.stop()

		# if the robot that is blocking this robot is not in turn being blocked by this robot, just wait for it to pass by
		if (self.blocked_by.blocked_by == None):
			# set this to true so that this robot will not attempy to re-route, instead commits to waiting
			self.is_waiting = True
			self.status = "Waiting"

		# else mexican stand-off, so one and only one robot should replan their route to avoid the other
		else:
			self.status = "Re-routing"
			# collect up all matrix positions to avoid
			avoid = []
			thisx, thisy = (self.map_grid.real_to_matrix(self.current_pose.pose.position.x, self.current_pose.pose.position.y))

			other_x, other_y = (self.map_grid.real_to_matrix(self.blocked_by.current_pose.pose.position.x, self.blocked_by.current_pose.pose.position.y))
			
			avoid = []
			pos = [other_x, other_y]
			# add all nodes in a large area around the blocking robot
			capturearea = 10
			for x in range (-capturearea,capturearea):
				for y in range (-capturearea,capturearea):
					newx = pos[0] + x
					newy = pos[1] + y
					if (newx >= 0 and newx < self.map_grid.width):
						if (newy >= 0 and newy < self.map_grid.height):
							avoid.append([newy, newx])



			thisx, thisy = (self.map_grid.real_to_matrix(self.current_pose.pose.position.x, self.current_pose.pose.position.y))
			this_pos = [thisx, thisy]
			capturearea = 10
			for x in range (-capturearea,capturearea):
				for y in range (-capturearea,capturearea):
					newx = this_pos[0] + x
					newy = this_pos[1] + y
					if (newx >= 0 and newx < self.map_grid.width):
						if (newy >= 0 and newy < self.map_grid.height):
							if [newy, newx] in avoid:
								avoid.remove([newy, newx])

			self.blocked_nodes = avoid

			# now call pathfinding with this list of nodes to avoid
			startx, starty = self.map_grid.real_to_matrix(self.current_pose.pose.position.x, self.current_pose.pose.position.y)
			s = Point(startx, -starty, 0)

			f = self.path_to_next_item[len(self.path_to_next_item)-1]

			# if the target position is within the avoid array
			if f in avoid:
				self.path_to_next_item = self.new_path()
				return

			e = Point(f[1], -f[0],0)


			self.path_to_next_item = self.trim_path(self.A_star(s,e, avoid))

			if (len(self.path_to_next_item) == 0):
				# can't reach target, so just get nee random one
				self.path_to_next_item = self.new_path()
			else:
				self.status = "Succesful re-route"

			# set to None so that the robot that is blocking this one knows to just wait instead of replan route
			self.blocked_by = None
									
# ----------------------------------------------------------------------------

	def distance(self, p1, p2):
		return math.sqrt( (p2.x - p1.x)**2 + (p2.y - p1.y)**2 )

	def heuristic(self, p1, p2):
		return self.distance(p1, p2)

	def A_star(self, start_position, target_position, avoid):
		"""
		Find shortest path from a given start "position" to 
		a given target "position" (using A* algorithm)
		Return: Array of pixel "coordinates" the robot would visit

		EXTRA: the parameter 'avoid' is an array of matrix positions that a* should avoid
		"""
		Nodes = self.map_grid.gridNodes
		startNodeIndex = None
		endNodeIndex = None

		lowestDist = 9999999999

		startx, starty = self.map_grid.real_to_matrix(self.current_pose.pose.position.x, self.current_pose.pose.position.y)

		#Reset all nodes' parent, visited and distance values
		for n in range(0, len(Nodes)):
			Nodes[n].visited = False
			Nodes[n].lDist = 999999999999999999
			Nodes[n].gDist = 999999999999999999
			Nodes[n].parent = None
			#Set starting and ending nodes

			# robots can sometimes get out of sync with matrix, which means they are slightly off the map/ inside a wall
			# by choosing the nearest start node instead of the exact one, can solve this issue
			dist = math.sqrt( (Nodes[n].p.x - startx)**2 + (Nodes[n].p.y - -starty)**2 )
			if (dist < lowestDist):
				lowestDist = dist
				startNodeIndex = n

			#if (Nodes[n].p == start_position): 
			#	startNodeIndex = n

			if (Nodes[n].p == target_position): 
				endNodeIndex = n

			# if the current node is in the avoid list, mark it as already visited
			# this will mean the algorithm does ignore's it as a viable
			current = [Nodes[n].p.x, Nodes[n].p.y]
			for pos in avoid:
				# x and y must be flipped
				if (Nodes[n].p.x == pos[1] and Nodes[n].p.y == -pos[0]):
					Nodes[n].visited = True
					break

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
					# check each position in the avoid array, if any match the current node, treat
					# this node as an obstacle
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
				path.append([y,x])
				cNI = Nodes[cNI].parent
			path.reverse()

		#self.map_grid.output_rgb_path_image(self.map_grid.gridArray, path, 'a.png')

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
