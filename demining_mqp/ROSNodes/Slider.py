#!/usr/bin/env python
import rospy
import numpy as np
import RPi.GPIO as GPIO
from demining_mqp.msg import*

class slider:
    def __init__(self):
        self._sendSliderPos = rospy.Publisher('/sliderPos', sliderposition, queue_size=1)# need to make message type
        self._receiveCommand = rospy.Subscriber('/sliderCommand', slidercommand, self.handleCommand, queue_size=4)#need to make message type
        self.LeftLimitSwitchPin = 40
        self.RightLimitSwitchPin = 38
        self.MotorDirectionPin = 36
        self.MotorSpeedPin = 37
        #self.MotorStallPin = 20
        self.CurrentMotorDirection = 1 # 0 for left, 1 for right? whatever you want
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.RightLimitSwitchPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.LeftLimitSwitchPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        #GPIO.setup(self.MotorStallPin, GPIO.IN)
        GPIO.setup(self.MotorDirectionPin, GPIO.OUT)
        GPIO.setup(self.MotorSpeedPin, GPIO.OUT)
        self.ScanFreely = True
        self.stepCount = 0
        self.stepDir = ""
        self.totalStepCount = 0
        self.leftBound = 0
        self.rightBound = 0
        self.speed = .006

    def zero(self):
        GPIO.output(self.MotorDirectionPin, self.CurrentMotorDirection)

        while(GPIO.input(self.LeftLimitSwitchPin) == GPIO.HIGH):
            GPIO.output(self.MotorSpeedPin, GPIO.HIGH)
            rospy.sleep(self.speed)
            GPIO.output(self.MotorSpeedPin, GPIO.LOW)
            rospy.sleep(self.speed)

        self.totalStepCount = 0
        self.CurrentMotorDirection = not self.CurrentMotorDirection
        rospy.sleep(.01)

        while (GPIO.input(self.RightLimitSwitchPin) == GPIO.HIGH):
            GPIO.output(self.MotorSpeedPin, GPIO.HIGH)
            rospy.sleep(self.speed)
            GPIO.output(self.MotorSpeedPin, GPIO.LOW)
            rospy.sleep(self.speed)
            self.totalStepCount += 1

        self.CurrentMotorDirection = not self.CurrentMotorDirection
        self.stepCount = self.totalStepCount
        self.leftBound = 0 + 100
        self.rightBound = self.totalStepCount - 100
        rospy.sleep(.01)

        while (not self.stepCount == self.leftBound):
            GPIO.output(self.MotorSpeedPin, GPIO.HIGH)
            rospy.sleep(self.speed)
            GPIO.output(self.MotorSpeedPin, GPIO.LOW)
            rospy.sleep(self.speed)
            self.stepCount -= 1

        self.CurrentMotorDirection = not self.CurrentMotorDirection

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
            if(GPIO.input(self.RightLimitSwitchPin) == GPIO.LOW):
                self.CurrentMotorDirection = 1
            elif(GPIO.input(self.LeftLimitSwitchPin) == GPIO.LOW):
                self.CurrentMotorDirection = 0
                #Dillon have motor switch directions appropriately and move a certain number of ticks.
                #also publish a ROS Message with the new data

            GPIO.output(self.MotorSpeedPin, GPIO.HIGH)
            rospy.sleep(self.speed)
            GPIO.output(self.MotorSpeedPin, GPIO.LOW)
            rospy.sleep(self.speed)
            if (self.CurrentMotorDirection == 1):
                self.stepCount -= 1
            elif (self.CurrentMotorDirection == 0):
                self.stepCount += 1

            if(self.stepCount == self.leftBound):
                self.CurrentMotorDirection = 0
            elif(self.stepCount == self.rightBound):
                self.CurrentMotorDirection = 1

            if(GPIO.input(self.RightLimitSwitchPin) == GPIO.LOW and GPIO.input(self.LeftLimitSwitchPin) == GPIO.LOW):
                self.ScanFreely = not self.ScanFreely


            #have arm move a certain number of steps and publish position
        #self._sendSliderPos.publish(self, self.stepCount, self.stepDir) #example of publishing message

if __name__ == '__main__':
    rospy.init_node('SliderControl')
    sliderControl = slider()
    rospy.sleep(1)
    sliderControl.zero()

    while not rospy.is_shutdown():
        rospy.sleep(0.02)
        sliderControl.scan()