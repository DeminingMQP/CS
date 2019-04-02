#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from demining_mqp.msg import *

def talker():
    pub = rospy.Publisher('maps', mapsAPI, queue_size=10)
    rospy.sleep(0.5)
    valueFile = open("values.txt","r")
    xCoorFile = open("xCoor.txt","r")
    yCoorFile = open("yCoor.txt","r")
    num = ''
    myMap = mapsAPI()
    for line in valueFile:
        row = array1d()
        myNode = mapNode()
        for letter in line: 
            if (letter == ' '):
                myNode.value=int(num)
                row.array.append(myNode)                
                num = ''
                myNode = mapNode()
            elif(letter=='\n'):
                _=0
            else:
                num+=letter
        myMap.grid.append(row)
    rowIndex = 0
    columnIndex = 0
    for line in xCoorFile:
        for letter in line:
            if (letter == ' '):
                print(rowIndex,columnIndex)
                myMap.grid[columnIndex].array[rowIndex].gps.x=float(num)            
                num = ''
                rowIndex+=1
            elif(letter=='\n'):
                _=0
            else:
                num+=letter
        rowIndex=0
        columnIndex +=1
    rowIndex = 0
    columnIndex = 0
    for line in yCoorFile:
        for letter in line:
            if (letter == ' '):
                myMap.grid[columnIndex].array[rowIndex].gps.y=float(num)                  
                num = ''
                rowIndex+=1
            elif(letter=='\n'):
                _=0
            else:
                num+=letter
        rowIndex=0
        columnIndex +=1
    print(myMap)
    pub.publish(myMap)


if __name__ == '__main__':
    rospy.init_node('baseStationMap', anonymous=True)
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
