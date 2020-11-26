"""
map_grid.py
Provides a MapGrid class to calculate particle weights.
"""
import rospy

import math

#import laser_trace

from    geometry_msgs.msg   import Pose, PoseArray, Quaternion, PoseWithCovarianceStamped
from 	nav_msgs.msg 		import OccupancyGrid, Odometry
from    util            import rotateQuaternion, getHeading
from math import sin, cos, radians, pi, degrees

PI_OVER_TWO = math.pi/2 

class MapGrid(object):
    def __init__(self):
        # ----- Map data
        self.width = 0.0
        self.height = 0.0
        self.resolution = 0.0
        self.data =  OccupancyGrid.data
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.list = []


    def set_map(self, occupancy_map):
    
        self.width = occupancy_map.info.width
        self.height = occupancy_map.info.height
        self.resolution = occupancy_map.info.resolution # in m per pixel
        self.data =  occupancy_map.data 
        self.origin_x = ( occupancy_map.info.origin.position.x + (self.width / 2.0) * self.resolution )
        self.origin_y = ( occupancy_map.info.origin.position.y + (self.height / 2.0) * self.resolution )
        rospy.loginfo("Map set.")

        inGrid = list(occupancy_map.data)
        gridAsListOfLists = []
        #0 = white, 100 = black, -1 = grey      

        for row in range(0,self.height):
            temprow = []
            for col in range(0,self.width):
                element = inGrid.pop(0)
                temprow.append(element)
                gridAsListOfLists.append(temprow)
        self.list = gridAsListOfLists
        rospy.loginfo("Map as grid set.")
    

