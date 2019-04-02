#!/usr/bin/env python
import rospy, math
from std_msgs.msg import Bool
import numpy as np
import RPi.GPIO as GPIO
import fourbarposition
from demining_mqp.msg import*


class fourbar:
    UP = True
    DOWN = False

    def __init__(self):
        self._sendFourBarData = rospy.Publisher('/FourBarPos', fourbarposition,
                                                queue_size=5)
        self.metalDetectorListener = rospy.Subscriber('/calibrate', bool, self.calibrateCallback)
        # send to nav system with a new message type
        self._sendObstacle = rospy.Publisher('/SteepSlope', Bool, queue_size=1)
        self.upPin = 12
        self.downPin = 11
        self.Motor1EncPin1 = 15
        self.Motor1EncPin2 = 18
        self.Motor1SpeedPin = 16
        self.Motor1DirPin = 13
        self.Motor2EncPin1 = 31
        self.Motor2EncPin2 = 29
        self.Motor2SpeedPin = 32
        self.Motor2DirPin = 33
        GPIO.setmode(GPIO.Board)
        GPIO.setup(self.upPin, GPIO.IN)  # need to pick a pinbases
        GPIO.setup(self.downPin, GPIO.IN)  # need to pick a pin
        GPIO.setup(self.Motor1DirPin, GPIO.OUT)
        GPIO.setup(self.Motor2DirPin, GPIO.OUT)

        GPIO.add_event_detect(15, GPIO.RISING, callback=self.motor1Enc1Callback(), bouncetime=5)
        GPIO.add_event_detect(18, GPIO.RISING, callback=self.motor1Enc2Callback(), bouncetime=5)
        GPIO.add_event_detect(31, GPIO.RISING, callback=self.motor2Enc1Callback(), bouncetime=5)
        GPIO.add_event_detect(33, GPIO.RISING, callback=self.motor2Enc2Callback(), bouncetime=5)
        
        self.pwm1 = GPIO.PWM(16, 50)
        self.pwm2 = GPIO.PWM(32, 50)
        self.pwm1.start(0)
        self.pwm2.start(0)

        self.m1e1Value = 0
        self.m1e2Value = 0
        self.m2e1Value = 0
        self.m2e2Value = 0
        self.m1dir = 1
        self.fbp = fourbarposition(8, 40.76, 14.83, 24, 5, 3)

    def checkAlign(self):
        # Copy encoder values
        m1Enc = self.m1e1Value
        m2Enc = self.m2e1Value
        tolerance = 10
        # Check if
        if abs(m1Enc - m2Enc) > tolerance:
            if m2Enc > m1Enc:
                self.pwm2.stop()
                GPIO.output(self.Motor1DirPin, UP)
                self.pwm1.ChangeDutyCycle(50)
                self.checkAlign()
            elif m1Enc > m2Enc:
                self.pwm1.stop()
                GPIO.output(self.Motor2DirPin, UP)
                self.pwm2.ChangeDutyCycle(50)
                self.checkAlign()

    def checkPins(self):
        upPin = GPIO.input(self.upPin)
        downPin = GPIO.input(self.downPin)
        if upPin and downPin:
            self._sendObstacle.publish(True)
            self.pwm1.stop()
            self.pwm2.stop()
        elif upPin:
            self.m1dir = UP
            GPIO.output(self.Motor1DirPin, self.m1dir)
            GPIO.output(self.Motor2DirPin, not self.m1dir)
            self.pwm1.ChangeDutyCycle(50)
            self.pwm2.ChangeDutyCycle(50)
        elif downPin:
            self.m1dir = DOWN
            GPIO.output(self.Motor1DirPin, self.m1dir)
            GPIO.output(self.Motor2DirPin, not self.m1dir)
            self.pwm1.ChangeDutyCycle(50)
            self.pwm2.ChangeDutyCycle(50)
        else:
            self.pwm1.stop()
            self.pqm2.stop()
        self.fbp.calcPos(self.m1e1Value)
        self._sendFourBarData(self.fbp)


    def motor1Enc1Callback(self):
        self.m1e1Value += self.m1dir * 1

    def motor1Enc2Callback(self):
        self.m1e2Value += self.m1dir * 1

    def motor2Enc1Callback(self):
        self.m2e1Value += self.m2dir * 1

    def motor2Enc2Callback(self):
        self.m2e2Value += self.m2dir * 1

    def calibrateCallback(self):
        # set the encoder values to 0 when calibrate message is received
        self.m1e1Value = 0
        self.m1e2Value = 0
        self.m2e1Value = 0
        self.m2e2Value = 0


if __name__ == '__main__':
    rospy.init_node('FourBarControl')
    FourBarControl = fourbar()
    rospy.sleep(1)
    then = rospy.Time.now()

    while not rospy.is_shutdown():
        rospy.sleep(0.02)
        now = rospy.Time.now()
        if (now.secs - then.secs) > 10:
            FourBarControl.checkAlign()
            then = now
        FourBarControl.checkPins()
