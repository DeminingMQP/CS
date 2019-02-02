#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
import numpy as np
import RPi.GPIO as GPIO

#0 no landmine
#1 landmine MD
#2 landmine IR
#3 landmine Both
class landmineDet:
    def __init__(self):
        self._sendLandmineStatus = rospy.Publisher('/LandmineDetected', np.uint8t, queue_size=5)# send to nav system
        self._receiveIRData = rospy.Subscriber('/IRLandmineDet', Bool, self.updateIRData, queue_size=1 )
        self.currentIRReadings = False
        GPIO.setmode(GPIO.Board)
        GPIO.setup(7, GPIO.IN)
    def checkMine(self):
        if GPIO.input(7):
            if self.currentIRReadings is True:
                self._sendLandmineStatus.publish(3)
            else:
                self._sendLandmineStatus(1)
        else:
            if self.currentIRReadings is True:
                self._sendLandmineStatus(2)
            else:
                self._sendLandmineStatus(0)
    def updateIRData(self, data):
        self.currentIRReadings = data
if __name__ == '__main__':
    rospy.init_node('LandmineDetection')
    LandmineDetector = landmineDet()
    rospy.sleep(1)

    while not rospy.is_shutdown():
       rospy.sleep(0.02)
       LandmineDetector.checkMine()
