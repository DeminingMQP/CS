import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
from geometry_msgs.msg import Point
import sys
import select
from copy import deepcopy
from demining_mqp.msg import*
import math
import std_msgs

class Navigator:
    def __init__(self):
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.slider_pub = rospy.Publisher('/sliderCommand',slidercommand,queue_size = 10)
        self.sensorArm_pub = rospy.Publisher('/NavCommand', std_msgs.msg.Int8, queue_size=5)
        self.mineLocated_pub = rospy.Publisher('/mineLocation', Point, queue_size=5) 
        self.heading = 0
        self.roll = 0
        self.pitch = 0
        self.currentGPSLocation = Point()
        self.currentGPSLocation.x = 0
        self.currentGPSLocation.y = 0
        self.startTurn = True
        self.sliderPosition = 0
        self.sliderDirection = 0
        self.sliderMin = 0
        self.sliderMax = 0
        self.landmineDetected = False
        self.vel_msg = Twist()
        self.vel_msg.linear.x = 0
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = 0
        self.currDirection = 2 #starts facing right
        self.currLocation = (2,4)
        self.grid = []
        self.halfSizeOfRobot = 2
        self.dgps_sub = rospy.Subscriber('/dgps_location',Point, self.updateLocation)
        self.odom_sub = rospy.Subscriber('/husky_velocity_controller/odom',Odometry, self.updateEnc)
        self.slider_sub = rospy.Subscriber('/sliderPos',sliderposition,self.updateSliderPosition)
        self.IMU_sub = rospy.Subscriber('/IMU',IMUOrientation,self.updateIMUOrientation)
        self.map_sub = rospy.Subscriber('/maps',mapsAPI, self.makeMap)
        self.currentEnc = Point()
        self.currentEnc.x = 0
        self.currentEnc.y = 0
        self.currentEnc.z = 0
        self.gpsArr = [[50000000000,50000000000],[50000000000,50000000000],[50000000000,50000000000]]
        self.initialPos=[0,0]
        self.gridData = mapsAPI()
        self.gridReady = False
        self.gpsDiff = [0,0,0]
        

    def updateIMUOrientation(self,data):
        self.heading = data.heading
        self.roll = data.roll
        self.pitch = data.pitch

    def makeMap(self,data):
        print("in callback")
        self.gridData = data
        self.gridReady = True

    def readDetector(self,data):  
        if data ==1 or data ==3:
            self.landmineDetected = True
        else:
            self.landmineDetected = False
        

    def updateLocation(self,data):
        self.gpsArr[2] = deepcopy(self.gpsArr[1])
        self.gpsArr[1] = deepcopy(self.gpsArr[0])
        self.gpsArr[0][0]=data.y
        self.gpsArr[0][1]=data.x
        self.initialPos=[data.y,data.x]
        print(self.gpsArr)

    def updateSliderPosition(self,data):
        self.sliderPosition = data.motorstep
        self.sliderDirection = data.direction
        self.sliderMin = data.minStep
        self.sliderMax = data.maxStep

    def turnRight(self,speed):
        self.vel_msg.angular.z = -speed
        print("right")
        return

    def turnLeft(self,speed):
        self.vel_msg.angular.z = speed
        print("left")
        return 

    def driveStraight(self,speed):
        self.vel_msg.linear.x = speed
        return

    def fullStop(self):
        self.vel_msg.linear.x = 0
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = 0
        print("stop")
        return

    def updateEnc(self,data):
        self.currentEnc.x = data.pose.pose.position.x
        self.currentEnc.y = data.pose.pose.position.y
        self.currentEnc.z = data.pose.pose.orientation.z
        #print(currentEnc.z)

    def makeGridFromAPI(self):
        # 5 = unexplored
        #while self.gridReady==False:
        currTime = time.time()
        while self.gridReady == False:
            _=0

        print(self.gridReady)
        rospy.sleep(2)
        #print(self.gridReady)
        grid = []
        gridLength = len(self.gridData.grid)
        gridHeight = len(self.gridData.grid[0].array)
        self.initialPos[0]=42.27550245763326
        self.initialPos[0]=-71.80510319196128
        #while self.initialPos[0] == 0:
            #_=0
            #rospy.sleep(0.2)
        print(gridLength,gridHeight)
        for index in range (0,gridLength):
	        self.grid.append([])
	        for index2 in range(0,gridHeight):
	            self.grid[index].append({"value":self.gridData.grid[index].array[index2].value,"gps":{"x" : self.gridData.grid[index].array[index2].gps.x, "y" : self.gridData.grid[index].array[index2].gps.x}})
        locationSet = False
        while locationSet == False:
            gpsArr2 = deepcopy(self.gpsArr)
            self.gpsDiff[0] = math.sqrt(abs(gpsArr2[0][0]-gpsArr2[1][0])**2 + abs(gpsArr2[0][1]-gpsArr2[1][1])**2)
            self.gpsDiff[1] = math.sqrt(abs(gpsArr2[2][0]-gpsArr2[1][0])**2 + abs(gpsArr2[2][1]-gpsArr2[1][1])**2)
            self.gpsDiff[2] = math.sqrt(abs(gpsArr2[2][0]-gpsArr2[0][0])**2 + abs(gpsArr2[2][1]-gpsArr2[0][1])**2)
            print(len(self.grid),len(self.grid[0]))
            if (max(self.gpsDiff[0],self.gpsDiff[1],self.gpsDiff[2])<0.000003):
                print(self.gpsDiff)
                self.currentGPSLocation.x = gpsArr2[0][0]
                self.currentGPSLocation.y = gpsArr2[0][1]
                locDifference = self.currentGPSLocation.x + self.currentGPSLocation.y
                for gpsIndex in range(0,len(self.grid)):
                    for gpsIndex2 in range(0,len(self.grid[0])):
                        print(abs(abs(self.grid[gpsIndex][gpsIndex2]["gps"]["x"]-self.currentGPSLocation.x)+abs(self.grid[gpsIndex][gpsIndex2]["gps"]["y"]-self.currentGPSLocation.y)))
                        if (abs(abs(self.grid[gpsIndex][gpsIndex2]["gps"]["x"]-self.currentGPSLocation.x)+abs(self.grid[gpsIndex][gpsIndex2]["gps"]["y"]-self.currentGPSLocation.y))<locDifference and self.grid[gpsIndex][gpsIndex2]["value"] ==5):
                            locDifference = abs(self.grid[gpsIndex][gpsIndex2]["gps"]["x"]-self.currentGPSLocation.x)+abs(self.grid[gpsIndex][gpsIndex2]["gps"]["y"]-self.currentGPSLocation.y)
                            #print(locDifference)
                            closeLocation = (gpsIndex,gpsIndex2)   
                #self.currLocation = closeLocation
                locationSet=True 
        for index in range(self.currLocation[0]-self.halfSizeOfRobot,self.currLocation[0]+self.halfSizeOfRobot+1):
            for index2 in range(self.currLocation[1]-self.halfSizeOfRobot,self.currLocation[1]+self.halfSizeOfRobot+1):
                if(index>0 and index2>0 and index < len(self.grid) and index2<len(self.grid[0])):
                    self.grid[index][index2]["value"] = 2
        #grid[8][8] = 0
        return

    def makeGrid(self):
        # 5 = unexplored
        gridLength = 14
        gridHeight = 16
        #while self.initialPos[0] == 0:
        #    _=0
        for index in range (0,gridLength):
            self.grid.append([])
            for index2 in range(0,gridHeight):
                self.grid[index].append({"value":5, "gps" : {"x" : self.initialPos[1] + 0.000002, "y": self.initialPos[0] + 0.000002}})
        for index in range(self.currLocation[0]-self.halfSizeOfRobot,    self.currLocation[0]+self.halfSizeOfRobot+1):   
            for index2 in range(self.currLocation[1]-self.halfSizeOfRobot,    self.currLocation[1]+self.halfSizeOfRobot+1):                       
                self.grid[index][index2]["value"] = 2
        #grid[8][8] = 0
        return

    def pickTarget(self):
        return 0

    def determinePath(self,currTarget, pathType):
        targetDistance = 0
        unknownWeight = 5
        removeSpaces=[]
        targetX = 0
        targetY = 0
        potentialSpaces = [{"x":self.currLocation[0],"y":self.currLocation[1],"dist":0,"route":[], "target": 0}]
        archivedSpaces =[]
        targetRoute = []
        while targetRoute == []:
            newAdditions = []
            for potentialSpacesIndex in potentialSpaces:
                downRoute = potentialSpacesIndex["route"][:]
                downRoute.append(1)
                rightRoute = potentialSpacesIndex["route"][:]
                rightRoute.append(2)
                upRoute = potentialSpacesIndex["route"][:]
                upRoute.append(3)
                leftRoute = potentialSpacesIndex["route"][:]
                leftRoute.append(4)
                down = {"x":potentialSpacesIndex["x"],"y": potentialSpacesIndex["y"] - 1,"dist": potentialSpacesIndex["dist"] + 1,"route": downRoute, "target": 0}
                right = {"x":potentialSpacesIndex["x"] + 1,"y": potentialSpacesIndex["y"],"dist": potentialSpacesIndex["dist"] + 1,"route":  rightRoute, "target": 0}
                up = {"x":potentialSpacesIndex["x"],"y": potentialSpacesIndex["y"] + 1,"dist": potentialSpacesIndex["dist"] + 1,"route":  upRoute, "target": 0}
                left = {"x":potentialSpacesIndex["x"] - 1,"y": potentialSpacesIndex["y"],"dist": potentialSpacesIndex["dist"] + 1,"route":  leftRoute, "target": 0}
                currentAdjacentSpaces = [down, right, left, up]
                for index in currentAdjacentSpaces:
                    inArchive = 0
                    for archivedSpacesIndex in archivedSpaces:
                        if archivedSpacesIndex["x"] == index["x"] and archivedSpacesIndex["y"] == index["y"]:
                            inArchive = 1
                    if inArchive == 0:
                        if index["x"]-self.halfSizeOfRobot<0 or index["y"]-self.halfSizeOfRobot<0 or index["x"]+self.halfSizeOfRobot>len(self.grid) - 1 or index["y"]+self.halfSizeOfRobot>len(self.grid[0]) - 1 or self.grid[index["x"]][index["y"]]["value"] == 4 or self.grid[index["x"]][index["y"]]["value"] == 3 or self.grid[index["x"]][index["y"]]["value"] == 0 or self.grid[index["x"]][index["y"]]["value"] == 6:
                            index["dist"] = -1
                        elif self.grid[index["x"]][index["y"]]["value"] == 2:
                            match = 0
                            for index2 in newAdditions:
                                if index2["x"] == index["x"] and index2["y"] == index["y"] and index2["dist"] > index["dist"]:
                                    index2["dist"] = index["dist"]
                                    index2["route"] = index["route"]
                                    match = 1
                                elif index2["x"] == index["x"] and index2["y"] == index["y"] and index2["dist"] <= index["dist"]:
                                    match = 1
                            for index2 in potentialSpaces:
                                if index2["x"] == index["x"] and index2["y"] == index["y"] and index2["dist"] > index["dist"]:
                                    index2["dist"] = index["dist"]
                                    index2["route"] = index["route"]
                                    match = 1
                                elif index2["x"] == index["x"] and index2["y"] == index["y"] and index2["dist"] <= index["dist"]:
                                    match = 1
                            if match == 0:
                                newAdditions.append(index)
                        elif self.grid[index["x"]][index["y"]]["value"] == 5:
                            if pathType == "Frontier Path":
                                index["target"] = 1
                                newAdditions.append(index)
                                #unident this
                                """elif pathType == "Obstacle Avoidance":
                                index["dist"] += unknownWeight-1
                                for index2 in newAdditions:
                                    if index2["x"] == index["x"] and index2["y"] == index["y"] and index2["dist"] > index["dist"]:
                                        index2["dist"] = index["dist"]
                                        index2["route"] = index["route"]
                                        match = 1
                                    elif index2["x"] == index["x"] and index2["y"] == index["y"] and index2["dist"] <= index["dist"]:
                                        match = 1
                                for index2 in potentialSpaces:
                                    if index2["x"] == index["x"] and index2["y"] == index["y"] and index2["dist"] > index["dist"]:
                                        index2["dist"] = index["dist"]
                                        index2["route"] = index["route"]
                                        match = 1
                                    elif index2["x"] == index["x"] and index2["y"] == index["y"] and index2["dist"] <= index["dist"]:
                                        match = 1
                                if match == 0:
                                    newAdditions.append(index)
                            elif pathType == "Battery":
                                index["dist"] = -1"""
                            else:
                                print("Incorrect Path Type Error")
                        else:
                            print("Target Aquisition Error")
                        """if goalX == index["x"] and goalY == index["y"] and (pathType == "Battery" or pathType == "Obstacle Avoidance"):
                            index["target"] = 1"""
                    else:
		                u=0
                        #print("In Archive")
                archivedSpaces.append(potentialSpacesIndex)
                removeSpaces.append(potentialSpacesIndex)
            for newAdditionIndex in newAdditions:
                potentialSpaces.append(newAdditionIndex)
            for potentialSpacesIndex in potentialSpaces:
                if potentialSpacesIndex["target"] == 1:
                    targetRoute = potentialSpacesIndex["route"]
                    targetX = potentialSpacesIndex["x"]
                    targetY = potentialSpacesIndex["y"]
                    targetDistance = potentialSpacesIndex["dist"]
                    break
                elif potentialSpacesIndex["dist"] == -1:
                    removeSpaces.append(potentialSpacesIndex)
            for removeSpaceIndex in removeSpaces:
                potentialSpaces.remove(removeSpaceIndex)
            removeSpaces = []
            if potentialSpaces == []:
                return [], True
        print("here",targetX,targetY, self.currLocation)
        #time.sleep(2)
        return(targetRoute, False)

    def senseMine(self, sliderFirstReduced, sliderLastReduced):
        if self.currDirection == 1 and self.currLocation[1]-self.halfSizeOfRobot-1>0:
            for index in range(self.currLocation[0]-self.halfSizeOfRobot + min(sliderFirstReduced,sliderLastReduced),self.currLocation[0]-self.halfSizeOfRobot + max(sliderFirstReduced,sliderLastReduced)):
                self.grid[index][self.currLocation[1]-self.halfSizeOfRobot-1]["value"] = 3
                self.bufferSingleSquare((index,self.currLocation[1]-self.halfSizeOfRobot-1),self.grid)
        if self.currDirection == 2 and self.currLocation[0]+self.halfSizeOfRobot-+1<len(self.grid):
            for index in range(self.currLocation[1]-self.halfSizeOfRobot + min(sliderFirstReduced,sliderLastReduced),self.currLocation[1]-self.halfSizeOfRobot + max(sliderFirstReduced,sliderLastReduced)):
                self.grid[index][self.currLocation[0]-self.halfSizeOfRobot-1]["value"] = 3
                self.bufferSingleSquare((index,self.currLocation[0]+self.halfSizeOfRobot+1),self.grid)
        if self.currDirection == 3 and self.currLocation[1]+self.halfSizeOfRobot-+1<len(self.grid):
            for index in range(self.currLocation[0]-self.halfSizeOfRobot + min(sliderFirstReduced,sliderLastReduced),self.currLocation[0]-self.halfSizeOfRobot + max(sliderFirstReduced,sliderLastReduced)):
                self.grid[index][self.currLocation[1]-self.halfSizeOfRobot-1]["value"] = 3
                self.bufferSingleSquare((index,self.currLocation[1]+self.halfSizeOfRobot+1),self.grid)
        if self.currDirection == 4 and self.currLocation[0]-self.halfSizeOfRobot-1>0:
            for index in range(self.currLocation[1]-self.halfSizeOfRobot + min(sliderFirstReduced,sliderLastReduced),self.currLocation[1]-self.halfSizeOfRobot + max(sliderFirstReduced,sliderLastReduced)):
                self.grid[index][self.currLocation[0]-self.halfSizeOfRobot-1]["value"] = 3
                self.bufferSingleSquare((index,self.currLocation[0]-self.halfSizeOfRobot-1),self.grid)

    def bufferSingleSquare(self,location):
        for index in range (loacation[0]-self.halfSizeOfRobot,loaction[0]+self.halfSizeOfRobot+1): 
            for index2 in range (loacation[1]-self.halfSizeOfRobot,loaction[1]+self.halfSizeOfRobot+1):
                if index>0 and index2>0 and index<len(self.grid) and index2<len(self.grid[0]) and (self.grid[index][index2]["value"]==2 or self.grid[index][index2]["value"]==5):
                    self.grid[index][index2]["value"]=6
        return

    def establishBuffer(self):
        for index in range (0,len(self.grid)):
            for index2 in range (0,len(self.grid[0])):
                if index < self.halfSizeOfRobot or index2 < self.halfSizeOfRobot or index > len(self.grid)-self.halfSizeOfRobot-1 or index2 > len(self.grid[0])-self.halfSizeOfRobot-1:
                    if self.grid[index][index2]["value"] == 2 or self.grid[index][index2]["value"] == 5:
                        self.grid[index][index2]["value"] = 6
                if self.grid[index][index2]["value"] == 0:
                    for index3 in range(index-self.halfSizeOfRobot,index+self.halfSizeOfRobot+1):
                        for index4 in range(index2-self.halfSizeOfRobot,index2+self.halfSizeOfRobot+1):
                            if index3>0 and index4>0 and index3<len(self.grid)-1 and index4<len(self.grid[0])-1:
                                if self.grid[index3][index4]["value"] == 2 or self.grid[index3][index4]["value"] == 5:
                                    self.grid[index3][index4]["value"] = 6
        return

    def main(self):
        keyboardM = False
        keyboardP = False
        
        turnTime = 0
        newDirection = 0
        targetPath = []
        currTarget = (0,0)
        #currently leaves 2 width order
        self.makeGridFromAPI()
        #self.makeGrid()
        self.establishBuffer()
        maxSpeed = 0.5
        minSpeed = 0.1
        
        
        rate = rospy.Rate(10) # 10hz
        
        slider_msg = slidercommand()
        
        
        #landmineDetector_sub = rospy.Subscriber('/LandmineDetected',std_msgs.msg.Int8, readDetector, landmineDetected)
        startTime = time.time()
        stop = False
        mines = []
        pauseTime = 0
        pauseStart = -2
        prevEnc = Point()
        encDiff = Point()
        stage = 0
        targetPath, stop = self.determinePath((0,0), "Frontier Path")
        case = "explore"
        centered = False
        left = False
        error = False
        while not rospy.is_shutdown():
            try:
                if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
                    myInput = sys.stdin.read(1)
                    if myInput == " ":
                        mines.append(deepcopy(self.currentGPSLocation))
                        #print(mines)
                        dist = 0.3
                    if myInput == "m":
                        case = "detect mine"
                        minePhase = "scan forward"
                        firstSight = self.sliderPosition
                        firstDirection = self.sliderDirection
                        keyboardM = True
                    if myInput == "p":
                        keyboardP = True
                gpsArr2 = deepcopy(self.gpsArr)
                self.gpsDiff[0] = math.sqrt(abs(gpsArr2[0][0]-gpsArr2[1][0])**2 + abs(gpsArr2[0][1]-gpsArr2[1][1])**2)
                self.gpsDiff[1] = math.sqrt(abs(gpsArr2[2][0]-gpsArr2[1][0])**2 + abs(gpsArr2[2][1]-gpsArr2[1][1])**2)
                self.gpsDiff[2] = math.sqrt(abs(gpsArr2[2][0]-gpsArr2[0][0])**2 + abs(gpsArr2[2][1]-gpsArr2[0][1])**2)
                if (max(self.gpsDiff[0],self.gpsDiff[1],self.gpsDiff[2])<0.000003):
                    #print(gpsDiff)
                    self.currentGPSLocation.x = gpsArr2[0][0]
                    self.currentGPSLocation.y = gpsArr2[0][1]
                    locDifference = self.currentGPSLocation.x + self.currentGPSLocation.y
                    for gpsIndex in range(0,len(self.grid)):
                        for gpsIndex2 in range(0,len(self.grid[0])):
                            if ((abs(self.grid[gpsIndex][gpsIndex2]["gps"]["x"]-abs(self.currentGPSLocation.x))+abs(self.grid[gpsIndex][gpsIndex2]["gps"]["y"]-self.currentGPSLocation.y))<locDifference):
                                locDifference = abs(self.grid[gpsIndex][gpsIndex2]["gps"]["x"]-self.currentGPSLocation.x)+abs(self.grid[gpsIndex][gpsIndex2]["gps"]["y"]-self.currentGPSLocation.y)
                                #print(locDifference)
                                closeLocation = (gpsIndex,gpsIndex2)
                    #print(locDifference,grid[13][15]["gps"][0],grid[13][15]["gps"][1],self.currentGPSLocation,abs(abs(grid[gpsIndex][gpsIndex2]["gps"][0])-abs(self.currentGPSLocation.x)),abs(abs(grid[gpsIndex][gpsIndex2]["gps"][1])-abs(self.currentGPSLocation.y)),(abs(grid[gpsIndex][gpsIndex2]["gps"][0]-abs(self.currentGPSLocation.x))+abs(grid[gpsIndex][gpsIndex2]["gps"][1]-abs(self.currentGPSLocation.y)))<locDifference)   
                    #currLocation = closeLocation 
                    #self.targetPath, stop = self.determinePath(grid,(0,0),currLocation,halfSizeOfRobot, "Frontier Path")
                if case =="mark mine":
                    readyToExplore = False
                    if markPhase == "turning out":
                        self.sensorArm_pub.publish(6)
                        if sensorArmStatus == 9:
                            markPhase="paint"
                    elif markPhase == "paint":
                        time.sleep(2)
                        self.sensorArm_pub.publish(7)
                        markPhase = "turning in"
                    elif markphase == "turning in":
                        if sensorArmStatus == 0:
                            readyToExplore = True
                    if readyToExplore:
                        case = "explore" 
                elif case == "detect mine":           
                    if minePhase == "scan forward":
                        if self.sliderPosition == firstSight + 200 and keyboardM == True:
                            lastSight = self.sliderMax
                            minePhase = "centering"
                            keyboardM = False
                        elif self.sliderDirection != firstDirection:
                            lastSight = self.sliderMax
                            minePhase = "centering"
                        #elif not landmineDetected:
                            #lastSight = sliderPosition
                            #minePhase = "centering"
                    elif minePhase == "centering":
                        center = math.floor((firstSight + lastSight)/2)
                        slider_msg.scanFreely = False
                        slider_msg.motorstep = center
                        self.slider_pub.publish(slider_msg)
                        if self.sliderPosition == center:
                             centered = True
                    if centered:
                        sliderFirstReduced = math.floor(((2*self.halfSizeOfRobot+1)/(self.sliderMax-self.sliderMin+1)) * firstSight)
                        sliderLastReduced = math.floor(((2*self.halfSizeOfRobot+1)/(self.sliderMax-self.sliderMin+1)) * lastSight)
                        self.senseMine(self.halfSizeOfRobot,sliderFirstReduced,sliderLastReduced)
                        targetPath, stop = self.determinePath((0,0),self.halfSizeOfRobot, "Frontier Path")
                        centered = False                    
                        case ="mark mine"
                        markPhase = "turning out"
                elif case == "explore":
                    currTime = time.time()
                    encDiff.x = self.currentEnc.x - prevEnc.x
                    encDiff.y = self.currentEnc.y - prevEnc.y
                    encDiff.z = self.currentEnc.z - prevEnc.z
                    dist = math.sqrt(encDiff.x**2 + encDiff.y**2)
                    print(dist)
                    #currTarget = pickTarget(Grid, currLocation)
                    print(targetPath, self.currDirection)
                    if targetPath != []:
                        if pauseTime < currTime:
                            if ((targetPath[0]+1)%4 == self.currDirection%4):
                                if self.startTurn:
                                    self.startTurn = False
                                    targetHeading = (self.heading+90)%360
                                if abs(self.heading-targetHeading)>10 and keyboardP == False:
                                    self.turnRight(0.2)
                                else:
                                    keyboardP = False
                                    self.currDirection = targetPath[0]
                                    self.startTurn = True
                                    prevEnc = deepcopy(self.currentEnc)
                                    self.fullStop()
                                    pauseTime = currTime + 2
                            elif ((self.currDirection+1)%4 == targetPath[0]%4):
                                if self.startTurn:
                                    self.startTurn = False
                                    targetHeading = (self.heading-90)%360
                                if abs(self.heading-targetHeading)>10 and keyboardP == False:
                                    self.turnLeft(0.2)
                                else:
                                    keyboardP = False
                                    self.currDirection = targetPath[0]
                                    self.startTurn = True
                                    prevEnc = deepcopy(self.currentEnc)
                                    self.fullStop()
                                    pauseTime = currTime + 2
                            elif ((self.currDirection+2)%4 == targetPath[0]%4):
                                if self.startTurn:
                                    self.startTurn = False
                                    targetHeading = (self.heading+180)%360
                                if abs(self.heading-targetHeading)>10 and keyboardP == False:
                                    self.turnLeft(0.2)
                                else:
                                    keyboardP = False
                                    self.currDirection = targetPath[0]
                                    self.startTurn = True
                                    prevEnc = deepcopy(self.currentEnc)
                                    self.fullStop()
                                    pauseTime = currTime + 2
                            if self.currDirection == targetPath[0]:
                                if dist<0.2 and keyboardP == False:
                                    self.driveStraight(0.2)
                                else:
                                    keyboardP = False
                                    pauseTime = currTime + 2
                                    self.fullStop()
                                    prevEnc = deepcopy(self.currentEnc)                  
                                    if self.currDirection == 1:
                                        self.currLocation = (self.currLocation[0],self.currLocation[1]-1)
                                    if self.currDirection == 2:
                                        self.currLocation = (self.currLocation[0]+1,self.currLocation[1])
                                    if self.currDirection == 3:
                                        self.currLocation = (self.currLocation[0],self.currLocation[1]+1)
                                    if self.currDirection == 4:
                                        self.currLocation = (self.currLocation[0]-1,self.currLocation[1])
                                    targetPath, stop = self.determinePath((0,0), "Frontier Path")
                                    #for index in range(currLocation[0]-halfSizeOfRobot,    currLocation[0]+halfSizeOfRobot+1):   
                                        #for index2 in range(currLocation[1]-halfSizeOfRobot,    currLocation[1]+halfSizeOfRobot+1):                       
                                            #grid[index][index2] = 2
                            if self.currDirection == 1 and self.currLocation[1]-self.halfSizeOfRobot-1>0:
                                for index in range(self.currLocation[0]-self.halfSizeOfRobot,self.currLocation[0]+self.halfSizeOfRobot+1):
                                    if self.grid[index][self.currLocation[1]-self.halfSizeOfRobot-1] == 5:
                                        self.grid[index][self.currLocation[1]-self.halfSizeOfRobot-1] = 2
                            if self.currDirection == 2 and self.currLocation[0]+self.halfSizeOfRobot+2<len(self.grid):
                                print(self.currLocation, len(self.grid))
                                for index in range(self.currLocation[1]-self.halfSizeOfRobot,self.currLocation[1]+self.halfSizeOfRobot+1):
                                    if self.grid[self.currLocation[0]+self.halfSizeOfRobot+1][index]["value"] == 5:
                                        self.grid[self.currLocation[0]+self.halfSizeOfRobot+1][index]["value"] = 2
                            if self.currDirection == 3 and self.currLocation[1]+self.halfSizeOfRobot+2<len(self.grid[0]):
                                print(self.currLocation, len(self.grid[0]))
                                for index in range(self.currLocation[0]-self.halfSizeOfRobot,self.currLocation[0]+self.halfSizeOfRobot+1):
                                    if self.grid[index][self.currLocation[1]+self.halfSizeOfRobot+1]["value"] == 5:
                                        self.grid[index][self.currLocation[1]+self.halfSizeOfRobot+1]["value"] = 2
                            if self.currDirection == 4 and self.currLocation[0]-self.halfSizeOfRobot-1>0:
                                for index in range(self.currLocation[1]-self.halfSizeOfRobot,self.currLocation[1]+self.halfSizeOfRobot+1):
                                    if self.grid[self.currLocation[0]-self.halfSizeOfRobot-1][index]["value"] == 5:
                                        self.grid[self.currLocation[0]-self.halfSizeOfRobot-1][index]["value"] = 2
                        else:
                            self.fullStop()
                        for index in range(self.currLocation[0]-self.halfSizeOfRobot,self.currLocation[0]+self.halfSizeOfRobot):
                            for index2 in range(self.currLocation[1]-self.halfSizeOfRobot,self.currLocation[1]+self.halfSizeOfRobot):
                                if index >=0 and index2>=0 and index<len(self.grid) and index2<len(self.grid[0]) and self.grid[index][index2]["value"] == 5:
                                    self.grid[index][index2]["value"] = 2
                    if stop:
                        self.fullStop()
                        #print(mines)
                    self.vel_pub.publish(self.vel_msg)
                    rate.sleep()
                    print(" "),
                    for line in range(0,len(self.grid[0])):
                        print(line%10),
                    print()
                    for line in range(0,len(self.grid)):
                        print(line%10),
                        for num in range(0,len(self.grid[0])):
                            print(self.grid[line][num]["value"]),
                        print()
                    print("Loc:",self.currLocation)
                    if targetPath == []:
                        self.mineLocated_pub.publish(self.currentGPSLocation)
            except KeyboardInterrupt:
                #print(mines)
                exit(1)
            except Exception as e:
                print("error", e)
                time.sleep(1)
                exit(1)


if __name__ == '__main__':
    try:
        rospy.init_node('navigator', anonymous=True)
        rospy.sleep(0.5)
        navigator = Navigator()
        navigator.main()
    except rospy.ROSInterruptException:
        pass
