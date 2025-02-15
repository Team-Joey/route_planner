"""
map_grid.py
Provides a MapGrid class to calculate particle weights.
"""
import rospy

import  math
import  numpy as np
from    PIL import Image
import  csv
import  pickle

#import laser_trace

from    geometry_msgs.msg   import Pose, Point, PoseArray, Quaternion, PoseWithCovarianceStamped
from    nav_msgs.msg        import OccupancyGrid, Odometry
from    util            import rotateQuaternion, getHeading
from    math import sin, cos, radians, pi, degrees

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
        self.gridArray = np.array([])

    def get_map_as_lists(self, inList, height, width):

        rospy.loginfo("Creating abstract array from occupancy map data...")
        #0 = white, 100 = black, -1 = grey
        gridAsListOfLists = []
        #Create abstract array structure
        for row in range(0,self.height):
            temprow = []
            for col in range(0,self.width):
                #Keep popping elements from the occupancy grid list, add them to a temporary row
                element = inList.pop(0)
                temprow.append(element)
            #Append row to 'array'
            gridAsListOfLists.append(temprow)

        rospy.loginfo("Abstract array set.")

        return gridAsListOfLists

    def trim_map(self, gridList, height, width):

        #This method may need extension if we want to trim different size images
        rospy.loginfo("Setting map grid...")

        #Set as array
        gridAsArray = np.asarray(gridList)
        #Trim first and last row and column
        gridAsArray = np.delete(gridAsArray,height-1,0)
        gridAsArray = np.delete(gridAsArray,width-1,1)
        gridAsArray = np.delete(gridAsArray,0,0)
        gridAsArray = np.delete(gridAsArray,0,1)
        #rot90 because output somehow gets flipped
        #gridAsArray = np.rot90(gridAsArray)
        gridAsArray = np.flip(gridAsArray,0)

        self.OC_GRID_TEMP = gridAsArray

        #Update height and width (now that first+last rows+columns have been removed)
        self.height = height-2
        self.width = width-2

        rospy.loginfo("Map grid set.")

        return gridAsArray

    def output_greyscale_image_from_array(self, imageArray, outputDestination):

        img = Image.fromarray(np.uint8(imageArray * 255) , 'L')
        img.save(outputDestination)
        rospy.loginfo("Image {} saved.".format(outputDestination))

    def output_rgb_image_from_array(self, imageArray, outputDestination):
        coloursColumns = []

        for x in range(0, len(imageArray)):
            coloursRow = []
            for y in range(0, len(imageArray[0])):
                if imageArray[x][y] == 0:
                    row = (np.uint8(255), np.uint8(255), np.uint8(255), np.uint8(255))
                    coloursRow.append(row)
                elif imageArray[x][y] == 100:
                    row = (np.uint8(0), np.uint8(0), np.uint8(0), np.uint8(255))
                    coloursRow.append(row)
                elif imageArray[x][y] == -1:
                    row = (np.uint8(127), np.uint8(127), np.uint8(127), np.uint8(255))
                    coloursRow.append(row)
                elif imageArray[x][y] == 50:
                    row = (np.uint8(0), np.uint8(255), np.uint8(0), np.uint8(255))
                    coloursRow.append(row)
            coloursColumns.append(coloursRow)

        coloursArray = np.asarray(coloursColumns)
        coloursArray = np.reshape(coloursArray,(len(imageArray), len(imageArray[0]), 4))

        img = Image.fromarray(coloursArray)
        img.show()

    def output_rgb_path_image(self, imageArray, pathArray, outputDestination):
        coloursColumns = []

        for element in pathArray:
            imageArray[element[0]][element[1]] = 50 #x and y might need flipping

        for x in range(0, len(imageArray)):
            coloursRow = []
            for y in range(0, len(imageArray[0])):
                if imageArray[x][y] == 0:
                    row = (np.uint8(255), np.uint8(255), np.uint8(255), np.uint8(255))
                    coloursRow.append(row)
                elif imageArray[x][y] == 100:
                    row = (np.uint8(0), np.uint8(0), np.uint8(0), np.uint8(255))
                    coloursRow.append(row)
                elif imageArray[x][y] == -1:
                    row = (np.uint8(127), np.uint8(127), np.uint8(127), np.uint8(255))
                    coloursRow.append(row)
                elif imageArray[x][y] == 50:
                    row = (np.uint8(0), np.uint8(255), np.uint8(0), np.uint8(255))
                    coloursRow.append(row)
            coloursColumns.append(coloursRow)

        coloursArray = np.asarray(coloursColumns)
        coloursArray = np.reshape(coloursArray,(len(imageArray), len(imageArray[0]), 4))

        img = Image.fromarray(coloursArray)
        img.show()

    def reduce_resolution(self, gridArray, sf, ss):

        rospy.loginfo("Beginning resolution reduction...")

        lowerResGridAsListOfLists = []       #For low res map representation
        lowerResGridColorsAsListOfLists = [] #For low res image representation

        self.openNodes = []                       #For placing food objects in world
        self.walls = []

        scaleFactor = sf
        subgridSize = ss

        #For each row and column in the grid array...
        for i in range(0, len(gridArray) - subgridSize, scaleFactor):
            lowResRow = []
            lowResRowColors = []
            for j in range(0, len(gridArray[0]) - subgridSize, scaleFactor):
                nW = 0 #Count the number of white 'nodes'
                nB = 0 #Count the number of black 'nodes'
                nG = 0 #Count the number of grey 'nodes'

                #For each subgrid
                for x in range(0, subgridSize):
                    for y in range(0, subgridSize):
                        #If a node in the subgrid is of colour X, increment X count
                        if gridArray[i+y][j+x] == -1:
                                nG += 1
                        elif gridArray[i+y][j+x] == 0:
                            #If the node is white, we can place a food item here
                            #So keep a track of the positions of these nodes.
                            self.openNodes.append((i+y, j+x))
                            nW += 1
                        elif gridArray[i+y][j+x] == 100:
                            self.walls.append((i+y, j+x))
                            nB += 1

                #Find the simple majority of a subgrid
                colorValues = [nW,nB,nG]
                maxN = max(colorValues)
                maxIndex = colorValues.index(maxN)

                #Append appropriate value to lists
                if maxIndex == 0:
                    lowResRow.append(0)
                    lowResRowColors.append(1)
                elif maxIndex == 1:
                    lowResRow.append(100)
                    lowResRowColors.append(0)
                elif maxIndex == 2:
                    lowResRow.append(-1)
                    lowResRowColors.append(0.5)

                if (lowResRow[-1] == 0):
                    p = Point()
                    p.y = -i/scaleFactor
                    p.x = j/scaleFactor
                    tempNeighbours = []
                    k=0
                    l=0
                    u=0
                    #check for neighbours
                    for nd in range(len(self.gridNodes)-1,-1, -1):
                        #only check if node is not obstacle
                        if (not self.gridNodes[nd].isObstacle):
                            if (self.gridNodes[nd].p.x == (p.x-1) and self.gridNodes[nd].p.y == p.y):
                                tempNeighbours.append(nd)
                                self.gridNodes[nd].neighbours.append(len(self.gridNodes))
                                k=k+1
                                l=nd
                            elif (self.gridNodes[nd].p.x == p.x and self.gridNodes[nd].p.y == (p.y+1)):
                                tempNeighbours.append(nd)
                                self.gridNodes[nd].neighbours.append(len(self.gridNodes))
                                k=k+1
                                u=nd
                            elif (self.gridNodes[nd].p.x == (p.x-1) and self.gridNodes[nd].p.y == (p.y+1) and k==2):
                                tempNeighbours.append(nd)
                                self.gridNodes[nd].neighbours.append(len(self.gridNodes))
                                self.gridNodes[l].neighbours.append(u)
                                self.gridNodes[u].neighbours.append(l)
                            if (self.gridNodes[nd].p.x < (p.x-1) and self.gridNodes[nd].p.y > (p.y+1)): break
                    node = N(p,tempNeighbours,False)
                    self.gridNodes.append(node)

            #Append rows
            lowerResGridAsListOfLists.append(lowResRow)
            lowerResGridColorsAsListOfLists.append(lowResRowColors)

        pathList = []
        for node in self.gridNodes:
            coordinate = (-node.p.y, node.p.x) #This might need flipping
            pathList.append(coordinate)

        #Convert to numpy arrays
        lowResGridAsArray = np.asarray(lowerResGridAsListOfLists)
        lowResGridColorsAsArray = np.asarray(lowerResGridColorsAsListOfLists)
        openNodesAsArray = np.asarray(self.openNodes)
        pathArray = np.asarray(pathList)

        #rospy.loginfo("TESTING PART BEGINNING")
        #print("grid nodes len:", len(self.gridNodes), "path array len:",len(pathArray),"open nodes array len:",len(openNodesAsArray))

        #TESTING FOR COLOUR IMAGE OUTPUT
        for coordinate in pathArray:
            lowResGridAsArray[coordinate[0]][coordinate[1]] = 50 #THIS CAN BE CHANGED

        #IMPORTANT NOTE; lowResGridColorsAsArray contains greyscale intensities, not rgb values
        returnArray = np.array([lowResGridAsArray, lowResGridColorsAsArray, openNodesAsArray, pathList])

        rospy.loginfo("Resolution reduced.")

        return returnArray

    def reduce_resolution_weighted(self, gridArray, sf, ss, ww):
        rospy.loginfo("Beginning weighted resolution reduction...")

        lowerResGridAsListOfLists = []       #For low res map representation
        lowerResGridColorsAsListOfLists = [] #For low res image representation
        self.openNodes = []                       #For placing food objects in world
        self.walls = []

        scaleFactor = sf
        subgridSize = ss
        wallWeight = ww

        #For each row and column in the grid array...
        for i in range(0, len(gridArray) - subgridSize, scaleFactor):
            lowResRow = []
            lowResRowColors = []
            for j in range(0, len(gridArray[0]) - subgridSize, scaleFactor):
                nW = 0 #Count the number of white 'nodes'
                nB = 0 #Count the number of black 'nodes'
                nG = 0 #Count the number of grey 'nodes'

                #For each subgrid
                for x in range(0, subgridSize):
                    for y in range(0, subgridSize):
                        #If a node in the subgrid is of colour X, increment X count
                        if gridArray[i+y][j+x] == -1:
                                nG += 1
                        elif gridArray[i+y][j+x] == 0:
                            #If the node is white, we can place a food item here
                            #So keep a track of the positions of these nodes.
                            self.openNodes.append((i+y, j+x))
                            nW += 1
                        elif gridArray[i+y][j+x] == 100:
                            self.walls.append((i+y, j+x))
                            nB += 1

                #Find the simple majority of a subgrid
                colorValues = [nW,nB,nG]
                if(colorValues[1] >= wallWeight):
                    maxIndex = 1
                else:
                    maxN = max(colorValues)
                    maxIndex = colorValues.index(maxN)

                #Append appropriate value to lists
                if maxIndex == 0:
                    lowResRow.append(0)
                    lowResRowColors.append(1)
                elif maxIndex == 1:
                    lowResRow.append(100)
                    lowResRowColors.append(0)
                elif maxIndex == 2:
                    lowResRow.append(-1)
                    lowResRowColors.append(0.5)

                if (lowResRow[-1] == 0):
                    p = Point()
                    p.y = -i/scaleFactor
                    p.x = j/scaleFactor
                    tempNeighbours = []
                    k=0
                    l=0
                    u=0
                    #check for neighbours
                    for nd in range(len(self.gridNodes)-1,-1, -1):
                        #only check if node is not obstacle
                        if (not self.gridNodes[nd].isObstacle):
                            if (self.gridNodes[nd].p.x == (p.x-1) and self.gridNodes[nd].p.y == p.y):
                                tempNeighbours.append(nd)
                                self.gridNodes[nd].neighbours.append(len(self.gridNodes))
                                k=k+1
                                l=nd
                            elif (self.gridNodes[nd].p.x == p.x and self.gridNodes[nd].p.y == (p.y+1)):
                                tempNeighbours.append(nd)
                                self.gridNodes[nd].neighbours.append(len(self.gridNodes))
                                k=k+1
                                u=nd
                            elif (self.gridNodes[nd].p.x == (p.x-1) and self.gridNodes[nd].p.y == (p.y+1) and k==2):
                                tempNeighbours.append(nd)
                                self.gridNodes[nd].neighbours.append(len(self.gridNodes))
                                self.gridNodes[l].neighbours.append(u)
                                self.gridNodes[u].neighbours.append(l)
                            if (self.gridNodes[nd].p.x < (p.x-1) and self.gridNodes[nd].p.y > (p.y+1)): break
                    node = N(p,tempNeighbours,False)
                    self.gridNodes.append(node)

            #Append rows
            lowerResGridAsListOfLists.append(lowResRow)
            lowerResGridColorsAsListOfLists.append(lowResRowColors)

        #Convert to numpy arrays
        lowResGridAsArray = np.asarray(lowerResGridAsListOfLists)
        lowResGridColorsAsArray = np.asarray(lowerResGridColorsAsListOfLists)
        openNodesAsArray = np.asarray(self.openNodes)

        returnArray = np.array([lowResGridAsArray, lowResGridColorsAsArray, openNodesAsArray])

        rospy.loginfo("Resolution reduced.")

        return returnArray

    def expand_walls(self, gridArray, extensionValue):

        rospy.loginfo("Beginning wall expansion")

        wallCoordinates = []
        expandedArray = np.copy(gridArray)

        for i in range(0, len(gridArray)):
            for j in range(0, len(gridArray[0])):
                #If a wall is found in the true map
                if gridArray[i][j] == 100:
                    for x in range(-extensionValue, extensionValue+1):
                        for y in range(-extensionValue, extensionValue+1):
                            if(i+y>0 and i+y<len(gridArray) and j+x>0 and j+x<len(gridArray[0])):
                                expandedArray[i+y][j+x] = 100

        rospy.loginfo("Walls expanded")

        return expandedArray

    def set_map(self, occupancy_map):

        #TOGGLE BETWEEN WRITING AND READING CSV / .P
        writing = True

        #Set properties
        rospy.loginfo("Setting map properties...")
        self.width = occupancy_map.info.width
        self.height = occupancy_map.info.height
        self.resolution = occupancy_map.info.resolution # in m per pixel
        self.data =  occupancy_map.data
        self.origin_x = (occupancy_map.info.origin.position.x + (self.width / 2.0) * self.resolution)
        self.origin_y = (occupancy_map.info.origin.position.y + (self.height / 2.0) * self.resolution)
        rospy.loginfo("Map properties set.")

        self.resolution_reduction_scale = 3

        if(writing):

            #Convert row-major occupancy map data (one list) to a list of lists (abstract array structure)
            gridAsListOfLists = self.get_map_as_lists(list(occupancy_map.data), self.height, self.width)
            self.list = gridAsListOfLists

            #Reduced dimensions, more prime factors, different options of resolution reduction
            gridAsArray = self.trim_map(gridAsListOfLists, self.height, self.width)

            with open('ocgridtemp.csv', 'w') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerows(gridAsArray)

            gridAsArray = self.expand_walls(gridAsArray,10)
            infoArray = self.reduce_resolution_weighted(gridAsArray, self.resolution_reduction_scale, self.resolution_reduction_scale, 3)
            self.gridArray = infoArray[0]

            with open('gridarray.csv', 'w') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerows(self.gridArray)

            with open('walls.csv', 'w') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerows(self.walls)

            with open('opennodes.csv', 'w') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerows(self.openNodes)

            with open('gridnodes.p', 'wb') as picklefile:
                pickle.dump(self.gridNodes, picklefile)

        else:

            with open('gridarray.csv') as csvfile:
                lines = csvfile.readlines()
                array = []
                for line in lines:
                    line = line.replace("\n","")           #replace eol characters
                    line = line.split(",")                 #split on commas
                    newline = [int(item) for item in line] #list comprehension
                    array.append(newline)
                self.gridArray = np.asarray(array)

            with open('ocgridtemp.csv') as csvfile:
                lines = csvfile.readlines()
                array = []
                for line in lines:
                    line = line.replace("\n","")
                    line = line.split(",")
                    newline = [int(item) for item in line]
                    array.append(newline)
                self.OC_GRID_TEMP = np.asarray(array)

            with open('walls.csv') as csvfile:
                lines = csvfile.readlines()
                walls = []
                for line in lines:
                    line = line.replace("\n", "")
                    line = line.split(",")
                    newline = [int(item) for item in line]
                    walls.append(newline)
                self.walls = walls

            with open('opennodes.csv') as csvfile:
                lines = csvfile.readlines()
                openNodes = []
                for line in lines:
                    line = line.replace("\n", "")
                    line = line.split(",")
                    newline = [int(item) for item in line]
                    openNodes.append(newline)
                self.openNodes = openNodes

            with open('gridnodes.p', 'rb') as picklefile:
                self.gridNodes = pickle.load(picklefile)

    def rotate(self, origin, point, angle):
        ox, oy = origin
        px, py = point

        qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
        qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
        return qx, qy

    def real_to_matrix(self, x, y):
        """
        Rotate by 90 degrees, then undo the resolution scaling fators
        """

        newx, newy = self.rotate((self.origin_x,self.origin_y),(x,y), math.pi/2)

        newx = (newx / self.resolution) / self.resolution_reduction_scale
        newy = (newy / self.resolution) / self.resolution_reduction_scale

        return (math.ceil(newy), math.ceil(newx))

    def matrix_to_real(self, x, y):
        """
        Rotate by -90 degrees, then apply the resolution scaling fators
        """
        x *= self.resolution * self.resolution_reduction_scale
        y *= self.resolution * self.resolution_reduction_scale

        x, y = self.rotate((self.origin_x,self.origin_y),(x,y), -math.pi/2)

        return (x, y)
