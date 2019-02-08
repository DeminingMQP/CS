#!/usr/bin/env python

from graphics import *


def universalPathIdentifier(robotX, robotY, robotGrid, halfSizeOfRobot, goalX, goalY, pathType): #still will go as soon as least squares root is discoverd, even if it might not be least weight
    #print("Determining Path", goalX,goalY,robotX,robotY)
    targetDistance = 0
    unknownWeight = 5
    removeSpaces=[]
    targetX = 0
    targetY = 0
    potentialSpaces = [{"x":robotX,"y":robotY,"dist":0,"route":[], "target": 0}]
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
            #print(currentAdjacentSpaces)
            #print("potential", potentialSpaces)
            for index in currentAdjacentSpaces:
                inArchive = 0
                #print(index)
                for archivedSpacesIndex in archivedSpaces:
                    if archivedSpacesIndex["x"] == index["x"] and archivedSpacesIndex["y"] == index["y"]:
                        inArchive = 1
                if inArchive == 0:
                    #print(index)
                    if index["x"]-halfSizeOfRobot<0 or index["y"]-halfSizeOfRobot<0 or index["x"]+halfSizeOfRobot>len(robotGrid[0]) - 1 or index["y"]+halfSizeOfRobot>len(robotGrid[0]) - 1 or robotGrid[index["x"]][index["y"]] == 3 or robotGrid[index["x"]][index["y"]] == 0 or robotGrid[index["x"]][index["y"]] == 6:
                        index["dist"] = -1
                    elif robotGrid[index["x"]][index["y"]] == 2 or robotGrid[index["x"]][index["y"]] == 7:
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
                    elif robotGrid[index["x"]][index["y"]] == 5:
                        if pathType == "Frontier Path":
                            index["target"] = 1
                            newAdditions.append(index)
                        elif pathType == "Obstacle Avoidance":
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
                            index["dist"] = -1
                        else:
                            print("Incorrect Path Type Error")
                    else:
                        print("Target Aquisition Error")
                    if goalX == index["x"] and goalY == index["y"] and (pathType == "Battery" or pathType == "Obstacle Avoidance"):
                        index["target"] = 1
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
            return [],-1,-1, 0
    return(targetRoute,targetX,targetY, targetDistance)

def determineBatteryPath(robotX, robotY, goalX, goalY, robotGrid, halfSizeOfRobot):
    route = []
    #print("Battery Path")
    while len(route) == 0:
        route, targetX, targetY, distance = universalPathIdentifier(robotX, robotY, robotGrid, halfSizeOfRobot, goalX, goalY, "Battery")
    backRoute = []
    for i in route:
        backRoute.insert(0, i)
    return backRoute, goalX, goalY, distance

def determinePath(robotX, robotY, goalX, goalY, robotGrid, halfSizeOfRobot):
    distance = 1
    route = []
    while len(route) == 0:
        route, targetX, targetY, Disatnce = universalPathIdentifier(robotX, robotY, robotGrid, halfSizeOfRobot, goalX, goalY, "Obstacle Avoidance")
        distance = distance + 1
    backRoute = []
    for i in route:
        backRoute.insert(0, i)
    return backRoute, goalX, goalY

def obstacleAvoidingPath(obstacleAvoidance,targetX,targetY, robotGrid):
    if obstacleAvoidance == 1:
        targetY = targetY-1
    if obstacleAvoidance == 2:
        targetX = targetX+1
    if obstacleAvoidance == 4:
        targetX = targetX-1
    if obstacleAvoidance == 3:
        targetY = targetY+1
    if targetY != 0 and robotGrid[targetX][targetY - 1] == 5:
        return targetX, targetY - 1
    elif targetX != len(robotGrid)-1 and robotGrid[targetX + 1][targetY] == 5:
        return targetX + 1, targetY
    elif targetX != 0 and robotGrid[targetX - 1][targetY] == 5:
        return targetX - 1, targetY
    elif targetY != len(robotGrid[0])-1 and robotGrid[targetX][targetY + 1] == 5:
        return targetX, targetY + 1
    else:
        return -1, -1

def determineTargetSquare(direction,robotX, robotY, robotGrid, halfSizeOfRobot):
    distance = 1
    route = []
    while len(route) == 0:
        #route, targetX, targetY = targetSquareRecursion(distance, robotX, robotY, robotGrid)
        route, targetX, targetY, Distance = universalPathIdentifier(robotX, robotY, robotGrid, halfSizeOfRobot, 0, 0, "Frontier Path")
        if targetX == -1:
            return [], -1, -1
    backRoute = []
    for i in route:
        backRoute.insert(0,i)
    return backRoute, targetX,targetY;

def moveRobot(direction,robotX,robotY, robotGrid):
    print("move robit", direction)
    if direction == 1:
        print("IN 1", robotY,robotGrid[robotX][robotY-1])
        if robotGrid[robotX][robotY-1] == 2 or robotGrid[robotX][robotY-1] == 7:
            robotY = robotY-1
        elif robotGrid[robotX][robotY-1] == 3 or robotGrid[robotX][robotY-1] == 6:
            return 1,robotX,robotY
        #print(robotY)
    elif direction == 2:
        if robotGrid[robotX+1][robotY] == 2 or robotGrid[robotX+1][robotY] == 7:
            robotX = robotX+1
        elif robotGrid[robotX+1][robotY] == 3 or robotGrid[robotX+1][robotY] == 6:
            return 2,robotX,robotY
    elif direction == 3:
        if robotGrid[robotX][robotY+1] == 2 or robotGrid[robotX][robotY+1] == 7:
            robotY = robotY+1
        elif robotGrid[robotX][robotY+1] == 3 or robotGrid[robotX][robotY+1] == 6:
            return 3,robotX,robotY
    elif direction == 4:
        if robotGrid[robotX-1][robotY] == 2 or robotGrid[robotX-1][robotY] == 7:
            robotX = robotX-1
        elif robotGrid[robotX-1][robotY] == 3 or robotGrid[robotX-1][robotY] == 6:
            return 4,robotX,robotY
    else:
        return 5,robotX,robotY
    return 0,robotX,robotY

def robotDetector(robotX,robotY,robotGrid, masterGrid, halfSizeOfRobot):
    direction = 0
    for index in range(-halfSizeOfRobot, halfSizeOfRobot + 1):
        if robotY-halfSizeOfRobot-1 >= 0 and robotGrid[robotX + index][robotY-1-halfSizeOfRobot] == 5:
            #print("detect",robotGrid[robotX + index][robotY-1-halfSizeOfRobot])
            direction = 1
    for index in range(-halfSizeOfRobot, halfSizeOfRobot + 1):
        if direction == 0 and robotX+halfSizeOfRobot+1 <= len(robotGrid) - 1 and robotGrid[robotX + halfSizeOfRobot + 1][robotY+index] == 5:
            direction = 2
    for index in range(-halfSizeOfRobot, halfSizeOfRobot + 1):
        if direction == 0 and robotX-halfSizeOfRobot-1 >= 0 and robotGrid[robotX - 1 - halfSizeOfRobot][robotY+index] == 5:
            direction = 4
    for index in range(-halfSizeOfRobot, halfSizeOfRobot + 1):
        if direction == 0 and robotY+halfSizeOfRobot+1 <= len(robotGrid[0]) - 1 and robotGrid[robotX+index][robotY + 1 + halfSizeOfRobot] == 5:
            direction = 3
    for index in range(-halfSizeOfRobot, halfSizeOfRobot + 1):
        if direction == 1:
            detectY = robotY - 1 - halfSizeOfRobot
            detectX = robotX + index
        elif direction == 2:
            detectX = robotX + 1 + halfSizeOfRobot
            detectY = robotY + index
        elif direction == 3:
            detectY = robotY + 1 + halfSizeOfRobot
            detectX = robotX + index
        elif direction == 4:
            detectX = robotX - 1 - halfSizeOfRobot
            detectY = robotY + index
        else:
            return 0, robotGrid
        if detectX>=0 and detectX<=len(robotGrid)-1 and detectY>=0 and detectY<=len(robotGrid[0])-1 and direction != 0:

            robotGrid[detectX][detectY] = masterGrid[detectX][detectY]
            #print("here", detectX,detectY, robotGrid[detectX][detectY])
            if robotGrid[detectX][detectY] == 2 or robotGrid[detectX][detectY] == 7:
                for i in range(-halfSizeOfRobot + detectX,halfSizeOfRobot+detectX+1):
                    for j in range(-halfSizeOfRobot + detectY, halfSizeOfRobot + detectY + 1):
                        if i < 0 or j < 0 or j > len(robotGrid[0])-1 or i > len(robotGrid)-1:
                            robotGrid[detectX][detectY] = 6
                        elif robotGrid[i][j] == 0 or robotGrid[i][j] == 3:
                            robotGrid[detectX][detectY] = 6
            if robotGrid[detectX][detectY] == 3:
                for i in range(-halfSizeOfRobot + detectX,halfSizeOfRobot+detectX+1):
                    for j in range(-halfSizeOfRobot + detectY, halfSizeOfRobot + detectY + 1):
                        if i < 0 or j < 0 or j > len(robotGrid[0])-1 or i > len(robotGrid)-1:
                            print("do nothing")
                        elif robotGrid[i][j] == 2 or robotGrid[i][j] == 7:
                            robotGrid[i][j] = 6
    return direction,robotGrid


def defineMasterGrid():
    x = 5
    y = 4
    masterGrid = [[2] * y for _ in range(x)]
    masterGrid[2][2] = 3
    masterGrid[2][1] = 3
    masterGrid[2][0] = 3
    return masterGrid

def defineMasterGrid2():
    x = 30
    y = 30
    masterGrid = [[2] * y for _ in range(x)]
    for i in range (0,x):
        for j in range(0,y):
            if i+j>20:
                masterGrid[i][j] = 0
            if j<3:
                masterGrid[i][j] = 2
            if j>12 and j < 22 and i<20 and i > 10:
                masterGrid[i][j] = 2
            if i>11 and i<15 and j<18:
                masterGrid[i][j] = 2
            if j == 3 and i> 23 and i<27:
                masterGrid[i][j] = 2
            if j<6 and i < 19:
                masterGrid[i][j] = 2
            if j>13 and j<17 and i<17:
                masterGrid[i][j] = 2
            if i<3 and j < 21:
                masterGrid[i][j] = 2
    masterGrid[3][2] = 3
    masterGrid[3][1] = 3
    masterGrid[3][0] = 3
    masterGrid[0][6] = 3
    masterGrid[9][7] = 3
    masterGrid[28][1] = 3
    masterGrid[25][3] = 3
    masterGrid[15][15] = 3
    masterGrid[4][11] = 3
    masterGrid[8][12] = 0
    return masterGrid

def printMasterGrid():
    print("\n")
    #for i in range(len(masterGrid[0])-1,-1,-1):
        #for j in range(0,len(masterGrid)):
            #if j == robotX and i == robotY:
                #print("1"+" ", end="")
            #else:
                #print(str(masterGrid[j][i])+" ", end="")
        #print("\n")

def createGraphicGrid(robotGrid,win):
    graphicGrid = []
    for i in range(0, len(robotGrid)):
        graphicGrid.append([])
        for j in range(0, len(robotGrid[0])):
            graphicGrid[i].append(
                Rectangle(Point(50 + 10 * j - 5, 500 - 10 * i - 5), Point(50 + 10 * j + 5, 500 - 10 * i + 5)))
            graphicGrid[i][j].draw(win)
    return graphicGrid

def printRobotGrid(robotX,robotY, win, graphicGrid, halfSizeOfRobot, robotGrid,message):
    for i in range(len(robotGrid[0])-1,-1,-1):
       for j in range(0,len(robotGrid)):
           if 0:
               print()
           if j <= robotX+halfSizeOfRobot and j>=robotX-halfSizeOfRobot and i>=robotY-halfSizeOfRobot and i<=robotY+halfSizeOfRobot:
               #print("1"+" ", end="")
               graphicGrid[j][i].setFill('blue')
           else:
               #print(str(robotGrid[j][i])+" ", end="")
               if robotGrid[j][i] == 2:
                   graphicGrid[j][i].setFill('green')
               if robotGrid[j][i] == 3:
                   graphicGrid[j][i].setFill('red')
               if robotGrid[j][i] == 5:
                   graphicGrid[j][i].setFill('black')
               if robotGrid[j][i] == 6:
                   graphicGrid[j][i].setFill('orange')
               if robotGrid[j][i] == 7:
                   graphicGrid[j][i].setFill('gray')
               if robotGrid[j][i] == 0:
                   graphicGrid[j][i].setOutline('black')
                   graphicGrid[j][i].setFill('white')
    


def scanRobotGrid(robotGrid, win, unreachableSquare):
    if unreachableSquare == 1:
        message = Text(Point(win.getWidth() / 2, 20), 'Unreachable Squares')
        message.draw(win)
        time.sleep(10)
        return False
    for i in range(0,len(robotGrid)):
        for j in range(0,len(robotGrid[0])):
            if robotGrid[i][j] == 5:
                return True
    message = Text(Point(win.getWidth() / 2, 20), 'Minefield Cleared')
    message.draw(win)
    time.sleep(10)
    #input("Press Enter to continue...")
    return False

def scanDirection(direction, robotX, robotY, robotGrid,halfSizeOfRobot, masterGrid):
    print("Scan Direction")
    mine = False
    for index in range(-halfSizeOfRobot,halfSizeOfRobot+1):
        if direction == 1:
            detectX = robotX+index
            detectY = robotY-halfSizeOfRobot-1
        if direction == 2:
            detectY = robotY + index
            detectX = robotX + halfSizeOfRobot + 1
        if direction == 3:
            detectX = robotX + index
            detectY = robotY + halfSizeOfRobot + 1
        if direction == 4:
            detectY = robotY + index
            detectX = robotX - halfSizeOfRobot - 1
        if detectX >= 0 and detectX <= len(robotGrid) - 1 and detectY >= 0 and detectY <= len(robotGrid[0]) - 1:
            robotGrid[detectX][detectY]=masterGrid[detectX][detectY]
            print("scan",robotGrid[detectX][detectY])
            if robotGrid[detectX][detectY] == 2:
                for i in range(-halfSizeOfRobot + detectX,halfSizeOfRobot+detectX+1):
                    for j in range(-halfSizeOfRobot + detectY, halfSizeOfRobot + detectY + 1):
                        if i < 0 or j < 0 or j > len(robotGrid[0])-1 or i > len(robotGrid)-1:
                            robotGrid[detectX][detectY] = 6
                        elif robotGrid[i][j] == 0 or robotGrid[i][j] == 3:
                            robotGrid[detectX][detectY] = 6
            if robotGrid[detectX][detectY] == 3:
                for i in range(-halfSizeOfRobot + detectX,halfSizeOfRobot+detectX+1):
                    for j in range(-halfSizeOfRobot + detectY, halfSizeOfRobot + detectY + 1):
                        if i < 0 or j < 0 or j > len(robotGrid[0])-1 or i > len(robotGrid)-1:
                            print("do nothing")
                        elif robotGrid[i][j] == 2:
                            robotGrid[i][j] = 6
            if index == 0:
                if robotGrid[detectX][detectY] == 3:
                     mine = True
                elif robotGrid[detectX][detectY] == 0:
                    print("error2")
                else:
                    mine = False
    return mine, robotGrid

#if __name__=="__main__":
def navigate(masterGrid, robotGrid,chargingX,chargingY):
    halfSizeOfRobot = 1
    hasUnexploredTerritory = True
    hasValidMove = True
    #masterGrid = defineMasterGrid2()
    win = GraphWin("Minefield", 600, 600)
    #robotGrid, graphicGrid = defineRobotGrid2(win)
    graphicGrid = createGraphicGrid(robotGrid,win)
    #robotX, robotY = halfSizeOfRobot, halfSizeOfRobot
    robotGrid[chargingX][chargingY] = 7
    masterGrid[chargingX][chargingY] = 7
    robotX, robotY = chargingX,chargingY
    route = []
    obstacleAvoidance = 0
    targetX,targetY = 0,0
    maxBattery = 110
    batteryLife = maxBattery
    message = Text(Point(win.getWidth() / 2, 50), 'Battery Life ' + str(batteryLife))
    message.draw(win)
    unreachableSquares = 0
    batteryRoute, batteryTargetX, batteryTargetY, batteryDist = [],1,1,0
    for i in range(-halfSizeOfRobot + robotX, halfSizeOfRobot + robotX + 1):
        for j in range(-halfSizeOfRobot + robotY, halfSizeOfRobot + robotY + 1):
            if i<0 or j<0 or j>len(robotGrid[0])-1 or i > len(robotGrid)-1:
                print("do nothing")
            elif(i==chargingX and j == chargingY):
                print("also do nothing")
            elif i<halfSizeOfRobot or j<halfSizeOfRobot or i+halfSizeOfRobot>len(robotGrid)-1 or j+halfSizeOfRobot>len(robotGrid[0])-1:
                robotGrid[i][j] = 6
                #print("make 6")
            else:
                robotGrid[i][j] = 2
    printRobotGrid(robotX, robotY, win, graphicGrid, halfSizeOfRobot, robotGrid,message)
    #input("Press Enter to continue...")
    #print(hasUnexploredTerritory,hasValidMove)
    while hasUnexploredTerritory and hasValidMove:
        message.setText("Battery Life " + str(batteryLife))
        print("Location", robotX,robotY)
        if (robotX != chargingX or robotY!=chargingY):
            batteryRoute, batteryTargetX, batteryTargetY, batteryDist = determineBatteryPath(robotX, robotY, chargingX,chargingY ,robotGrid, halfSizeOfRobot)
            #print("Battery Dist",batteryDist)
        else:
            batteryLife = maxBattery
        if batteryLife <= batteryDist+2:
            route=batteryRoute
            targetX=batteryTargetX
            targetY=batteryTargetY
            #print(route)
        print("Calculating Move")
        if route==[]:

            direction, robotGrid = robotDetector(robotX,robotY,robotGrid, masterGrid,halfSizeOfRobot)
            printRobotGrid(robotX, robotY, win, graphicGrid, halfSizeOfRobot, robotGrid,message)
            #input("Press Enter to continue...")
            #print("route is empty", direction, route)
            if direction == 0:
                #print("No unexplored squares adjacent")
                route, targetX,targetY = determineTargetSquare(direction,robotX, robotY, robotGrid, halfSizeOfRobot)
                if route == []:
                    unreachableSquares = 1
                    hasUnexploredTerritory = scanRobotGrid(robotGrid, win, unreachableSquares)
                    break
        if route!=[]:
            print("route", route)
            direction = route[len(route)-1]
            #print("direction", direction)
            newMine, robotGrid = scanDirection(direction, robotX, robotY, robotGrid,halfSizeOfRobot,masterGrid)
            printRobotGrid(robotX, robotY, win, graphicGrid, halfSizeOfRobot, robotGrid,message)
            #("Press Enter to continue...")
            #print("new mine", newMine, route, direction, robotGrid[targetX][targetY])
            if robotGrid[targetX][targetY]==3 or robotGrid[targetX][targetY]==6:
                route = []
            else:
                if newMine:
                    route, targetX, targetY = determinePath(robotX, robotY,targetX, targetY, robotGrid, halfSizeOfRobot)
                    if route == []:
                        unreachableSquares = 1
                        hasUnexploredTerritory = scanRobotGrid(robotGrid, win, unreachableSquares)
                        break
                else:
                    print("option1")
                    obstacleAvoidance,robotX,robotY = moveRobot(direction,robotX,robotY, robotGrid)
                    del route[len(route)-1]
                print(direction, route)
        else:
            #print("direction", direction, robotX, robotY)
            print("option2")
            newMine, robotGrid = scanDirection(direction, robotX, robotY, robotGrid,halfSizeOfRobot,masterGrid)
            obstacleAvoidance,robotX,robotY = moveRobot(direction,robotX,robotY, robotGrid)
            #print("Avoiding", obstacleAvoidance, robotX, robotY)
        if obstacleAvoidance == 5:
            print("error")
        elif obstacleAvoidance != 0:
            targetX, targetY = obstacleAvoidingPath(obstacleAvoidance,robotX,robotY, robotGrid)
            if targetX == -1:
                route, targetX,targetY = determineTargetSquare(direction,robotX, robotY, robotGrid, halfSizeOfRobot)
                obstacleAvoidance = 0
            else:
                route, targetX, targetY = determinePath(robotX, robotY,targetX, targetY, robotGrid, halfSizeOfRobot)
        hasUnexploredTerritory = scanRobotGrid(robotGrid,win, unreachableSquares)
        printRobotGrid(robotX,robotY, win, graphicGrid, halfSizeOfRobot, robotGrid,message)
        #printMasterGrid()
        batteryLife-=1
        #print (batteryLife)
        #input("Press Enter to continue...")
    win.getMouse()
    win.close()
