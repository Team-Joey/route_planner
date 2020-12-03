"""
map_grid.py

"""
import rospy

import math
import numpy as np
from PIL import Image

#import laser_trace

from    geometry_msgs.msg   import Pose, Point, PoseArray, Quaternion, PoseWithCovarianceStamped
from 	nav_msgs.msg 		import OccupancyGrid, Odometry
from    util            import rotateQuaternion, getHeading
from math import sin, cos, radians, pi, degrees

PI_OVER_TWO = math.pi/2 

class N:
    def __init__(self, p, nb, v):
        self.parent = None
        self.p = p
        self.isObstacle = v
        self.visited = False
        self.lDist = 0.0
        self.gDist = 0.0
        self.neighbours = nb

class MapGrid(object):
    def __init__(self):
        # ----- Map data
        self.width = 0.0
        self.height = 0.0
        self.resolution = 0.0
        self.data =  OccupancyGrid.data
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.gridNodes = []

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
        
        #Reduced dimensions, more prime factors, different options of resolution reduction
        gridAsArray = np.asarray(gridAsListOfLists)
        gridAsArray = np.delete(gridAsArray,0,0)
        gridAsArray = np.delete(gridAsArray,0,1)
        gridAsArray = np.delete(gridAsArray,600,0)
        gridAsArray = np.delete(gridAsArray,600,1)
        gridAsArray = np.flip(gridAsArray,0)

        ##img = Image.fromarray(np.uint8(gridAsArray * 255) , 'L')
        ##img.save('main.png',format="png")
        lowerResGridAsListOfLists = []
        lowerResGridColorsAsListOfLists = []

        scaleFactor = 3
        openNodes = []

        for i in range(0,600,scaleFactor):
            lowResRow = []
            lowResRowColors = []
            for j in range(0,600,scaleFactor):
                nW = 0
                nB = 0
                nG = 0
                for x in range(0,scaleFactor):
                    for y in range(0,scaleFactor):
                        if gridAsArray[i+x][j+y] == -1:
                            nG += 1
                        elif gridAsArray[i+x][j+y] == 0:
                            openNodes.append((i+x,j+y))
                            nW += 1
                        elif gridAsArray[i+x][j+y] == 100:
                            nB += 1
                colorValues = [nW,nB,nG]
                maxN = max(colorValues)
                maxIndex = colorValues.index(maxN)
                if nB > 0:
                    lowResRow.append(100)
                    lowResRowColors.append(0)
                elif nG > 0:
                    lowResRow.append(-1)
                    lowResRowColors.append(0.5)
                elif maxIndex == 0:
                    lowResRow.append(0)
                    lowResRowColors.append(1)
                
                p = Point()
                p.y = -i/scaleFactor
                p.x = j/scaleFactor
                if (lowResRow[(len(lowResRow)-1)] == 0):
                    tempNeighbours = []
                    k=0
                    l=0
                    u=0
                    #check for neighbours
                    for nd in range(len(self.gridNodes)-1,-1, -1):
                        #only check if node is not obstacle
                        if (not self.gridNodes[nd].isObstacle):
                            if (self.gridNodes[nd].p.x == ((j/scaleFactor)-1) and self.gridNodes[nd].p.y == -(i/scaleFactor)):
                                tempNeighbours.append(nd)
                                self.gridNodes[nd].neighbours.append(len(self.gridNodes))
                                k=k+1
                                l=nd
                            elif (self.gridNodes[nd].p.x == (j/scaleFactor) and self.gridNodes[nd].p.y == -((i/scaleFactor)-1)):
                                tempNeighbours.append(nd)
                                self.gridNodes[nd].neighbours.append(len(self.gridNodes))
                                k=k+1
                                u=nd
                            elif (self.gridNodes[nd].p.x == ((j/scaleFactor)-1) and self.gridNodes[nd].p.y == -((i/scaleFactor)-1) and k==2):
                                tempNeighbours.append(nd)
                                self.gridNodes[nd].neighbours.append(len(self.gridNodes))
                                self.gridNodes[l].neighbours.append(u)
                                self.gridNodes[u].neighbours.append(l) 
                            if (self.gridNodes[nd].p.x < ((j/scaleFactor)-1) and self.gridNodes[nd].p.y > -((i/scaleFactor)-1)): break
                    node = N(p,tempNeighbours,False)
                    self.gridNodes.append(node)
                elif (lowResRow[(len(lowResRow)-1)] == 100):
                    self.gridNodes.append(N(p, [], True))
                       
            lowerResGridAsListOfLists.append(lowResRow)
            lowerResGridColorsAsListOfLists.append(lowResRowColors)

        #test
        #print(openNodes,len(openNodes))

        lowResGridAsArray = np.asarray(lowerResGridAsListOfLists)
        lowResGridColorsAsArray = np.asarray(lowerResGridColorsAsListOfLists)

        img = Image.fromarray(np.uint8(lowResGridColorsAsArray * 255) , 'L')
        img.save('test.png')

        ##lowResGridColorsAsArray2 = np.rot90(lowResGridColorsAsArray)

        ##img = Image.fromarray(np.uint8(lowResGridColorsAsArray2 * 255) , 'L')      
        ##img.save('test2.png')

        rospy.loginfo("Map as grid set.")
    

