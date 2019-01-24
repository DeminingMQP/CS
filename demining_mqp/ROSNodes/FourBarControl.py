#! /usr/bin/env python
import rospy, math
from std_msgs.msg import Bool
import numpy as np
import RPi.GPIO as GPIO
from demining_mqp.msg import*


class fourbar:
    def __init__(self):
        self._sendFourBarData = rospy.Publisher('/FourBarPos', fourbarposition,
                                                queue_size=5)  # send to nav system with a new message type
        self._sendObstacle = rospy.Publisher('/SteepSlope', Bool, queue_size=1)
        self.upPin = 13
        self.downPin = 14
        self.EncPin = 15
        self.MotorSpeedPin = 16
        self.MotorDirPin = 17
        GPIO.setmode(GPIO.Board)
        GPIO.setup(self.upPin, GPIO.IN)  # need to pick a pin
        GPIO.setup(self.downPin, GPIO.IN)  # need to pick a pin

    def checkPins(self):
        upPin = GPIO.input(self.upPin)
        downPin = GPIO.input(self.downPin)
        if upPin and downPin:
            self._sendObstacle.publish(True)
        elif upPin:
            # Add code to move up a magic number distance. Designed to move up until Sensor Platform doesnt need
            # it anymore. Once done delete the pass
            pass
        elif downPin:
            # Same idea except go down.Id recommend making these seperate functions
            pass
        else:
            pass


if __name__ == '__main__':
    rospy.init_node('FourBarControl')
    FourBarControl = fourbar()
    rospy.sleep(1)

    while not rospy.is_shutdown():
        rospy.sleep(0.02)
        FourBarControl.checkPins()
