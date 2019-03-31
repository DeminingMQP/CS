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
        self.CurrentMotorDirection = 1 # 0 for right, 1 for left
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
        self.speed = .004

    def zero(self):
        GPIO.output(self.MotorDirectionPin, self.CurrentMotorDirection)

        while(GPIO.input(self.LeftLimitSwitchPin) == GPIO.HIGH):
            GPIO.output(self.MotorSpeedPin, GPIO.HIGH)
            rospy.sleep(self.speed)
            GPIO.output(self.MotorSpeedPin, GPIO.LOW)
            rospy.sleep(self.speed)
            print "Homing Left"
        print "Button Pressed"
        self.totalStepCount = 0
        self.CurrentMotorDirection = 0
        GPIO.output(self.MotorDirectionPin, self.CurrentMotorDirection)
        rospy.sleep(.01)

        while (GPIO.input(self.RightLimitSwitchPin) == GPIO.HIGH):
            GPIO.output(self.MotorSpeedPin, GPIO.HIGH)
            rospy.sleep(self.speed)
            GPIO.output(self.MotorSpeedPin, GPIO.LOW)
            rospy.sleep(self.speed)
            self.totalStepCount += 1
            print "Homing Right"
        print "Button Pressed"
        self.CurrentMotorDirection = 1
        GPIO.output(self.MotorDirectionPin, self.CurrentMotorDirection)
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
            print "Returning to leftbound"

        self.CurrentMotorDirection = 0
        GPIO.output(self.MotorDirectionPin, self.CurrentMotorDirection)
        print "Homing Done"


    def handleCommand(self, data):
        if data.scanFreely is True:
            self.ScanFreely = True
        else:
            self.ScanFreely = False
            if data.motorstep is not -1:
                self.moveToStep(data.motorstep)

    def moveToStep(self, motorStep):

        if(self.stepCount <= motorStep):
            self.CurrentMotorDirection = 0
            GPIO.output(self.MotorDirectionPin, self.CurrentMotorDirection)
        elif(self.stepCount > motorStep):
            self.CurrentMotorDirection = 1
            GPIO.output(self.MotorDirectionPin, self.CurrentMotorDirection)

        while not(self.stepCount == motorStep):
            if (GPIO.input(self.RightLimitSwitchPin) == GPIO.LOW):
                print "!!!!!!!right limit switch reached"
                self.CurrentMotorDirection = 1
                GPIO.output(self.MotorDirectionPin, self.CurrentMotorDirection)
                motorStep = self.stepCount
            elif (GPIO.input(self.LeftLimitSwitchPin) == GPIO.LOW):
                print "!!!!!!!left limit switch reached"
                self.CurrentMotorDirection = 0
                GPIO.output(self.MotorDirectionPin, self.CurrentMotorDirection)
                motorStep = self.stepCount

            GPIO.output(self.MotorSpeedPin, GPIO.HIGH)
            rospy.sleep(self.speed)
            GPIO.output(self.MotorSpeedPin, GPIO.LOW)
            rospy.sleep(self.speed)

            if (self.CurrentMotorDirection == 1):
                self.stepCount -= 1
            elif (self.CurrentMotorDirection == 0):
                self.stepCount += 1

            if (self.stepCount == self.leftBound):
                print "left bound reached"
                self.CurrentMotorDirection = 0
                GPIO.output(self.MotorDirectionPin, self.CurrentMotorDirection)
                motorStep = self.stepCount
                GPIO.output(self.MotorSpeedPin, GPIO.LOW)
            elif (self.stepCount == self.rightBound):
                print "right bound reached"
                self.CurrentMotorDirection = 1
                GPIO.output(self.MotorDirectionPin, self.CurrentMotorDirection)
                motorStep = self.stepCount
                GPIO.output(self.MotorSpeedPin, GPIO.LOW)

        self._sendSliderPos.publish(self.stepCount, self.CurrentMotorDirection, self.leftBound, self.rightBound)
        

    def scan(self):
        if(self.ScanFreely):
            print "scanning"
            self._sendSliderPos.publish(self.stepCount, self.CurrentMotorDirection, self.leftBound, self.rightBound)
            if(GPIO.input(self.RightLimitSwitchPin) == GPIO.LOW):
                print "!!!!!!!right limit switch reached"
                self.CurrentMotorDirection = 1
                GPIO.output(self.MotorDirectionPin, self.CurrentMotorDirection)
            elif(GPIO.input(self.LeftLimitSwitchPin) == GPIO.LOW):
                print "!!!!!!!left limit switch reached"
                self.CurrentMotorDirection = 0
                GPIO.output(self.MotorDirectionPin, self.CurrentMotorDirection)

            GPIO.output(self.MotorSpeedPin, GPIO.HIGH)
            rospy.sleep(self.speed)
            GPIO.output(self.MotorSpeedPin, GPIO.LOW)
            rospy.sleep(self.speed)

            if (self.CurrentMotorDirection == 1):
                self.stepCount -= 1
            elif (self.CurrentMotorDirection == 0):
                self.stepCount += 1

            if(self.stepCount == self.leftBound):
                print "left bound reached"
                self.CurrentMotorDirection = 0
                GPIO.output(self.MotorDirectionPin, self.CurrentMotorDirection)
            elif(self.stepCount == self.rightBound):
                print "right bound reached"
                self.CurrentMotorDirection = 1
                GPIO.output(self.MotorDirectionPin, self.CurrentMotorDirection)

        if(GPIO.input(self.RightLimitSwitchPin) == GPIO.LOW and GPIO.input(self.LeftLimitSwitchPin) == GPIO.LOW):
            print "both switches triggered"
            self.ScanFreely = not self.ScanFreely
            rospy.sleep(2)


            #have arm move a certain number of steps and publish position
        #self._sendSliderPos.publish(self, self.stepCount, self.stepDir) #example of publishing message

if __name__ == '__main__':
    rospy.init_node('SliderControl')
    sliderControl = slider()
    rospy.sleep(1)
    sliderControl.zero()

    while not rospy.is_shutdown():
        sliderControl.scan()
