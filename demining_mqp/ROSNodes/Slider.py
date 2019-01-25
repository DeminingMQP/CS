#!/usr/bin/env python
import rospy
import numpy as np
import RPi.GPIO as GPIO
from demining_mqp.msg import*

class slider:
    def __init__(self):
        self._sendSliderPos = rospy.Publisher('/sliderPos', sliderposition, queue_size=1)# need to make message type
        self._receiveCommand = rospy.Subscriber('/sliderCommand', slidercommand, self.handleCommand(), queue_size=5)#need to make message type
        self.LeftLimitSwitchPin = 16
        self.RightLimitSwitch = 17
        self.MotorDirectionPin = 18
        self.MotorSpeedPin = 19
        self.MotorStallPin = 20
        self.CurrentMotorDirection = 1 # 0 for left, 1 for right? whatever you want
        GPIO.setmode(GPIO.Board)
        GPIO.setup(self.RightLimitSwitch, GPIO.IN)
        GPIO.setup(self.RightLimitSwitch, GPIO.IN)
        GPIO.setup(self.MotorStallPin, GPIO.IN)
        GPIO.setup(self.MotorDirectionPin, GPIO.OUTPUT)
        GPIO.setup(self.MotorSpeedPin, GPIO.OUTPUT)
        self.ScanFreely = True

    def handleCommand(self, data):
        if data.scanFreely is True:
            self.ScanFreely = True
        else:
            self.ScanFreely = False
        if data.motorstep is not 0:
            pass
            #dillon make this move to the motorstep position in the message

    #needs to be able to stop/start slider and command slider to move to a position and wait there like when a mine is
    #being marked
    def scan(self):
        if(self.ScanFreely):
            if(GPIO.input(self.RightLimitSwitch)):
                pass#Dillon have motor switch directions appropriately and move a certain number of ticks.
                    #also publish a ROS Message with the new data

            elif (GPIO.input(self.LeftLimitSwitch)):
                pass
            else:
                pass
            #have arm move a certain number of steps and publish position
        self._sendSliderPos.publish(1,1) #example of publishing message

if __name__ == '__main__':
    rospy.init_node('SliderControl')
    sliderControl = slider()
    rospy.sleep(1)

    while not rospy.is_shutdown():
        rospy.sleep(0.02)
        sliderControl.scan()