#!/usr/bin/env python
import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import *
import time


class maxbotix:
    def __init__(self):
        self._sendSensorData = rospy.Publisher('/Maxbotix', std_msgs.msg.Int16, queue_size=5)  # send to nav system
        self.risingPin = 10
        self.fallingPin = 11
        GPIO.setmode(GPIO.BOARD)

        self.riseCallback = GPIO.add_event_detect(self.risingPin, GPIO.RISING, self.risinghandle())
        self.fallCallback = GPIO.add_event_detect(self.fallingPin, GPIO.FALLING, self.fallinghandle())
        self.pulseStart = 0
        self.pulseEnd = 0
    def risinghandle(self):
        self.pulseStart = time.time()


    def fallinghandle(self):
        self.pulseEnd = time.time()
        cm = (self.pulseStart-self.pulseEnd)/10
        self._sendSensorData(cm)


if __name__ == '__main__':
    rospy.init_node('MaxbotixSensor')
    Sensor = maxbotix()
    rospy.sleep(1)

    while not rospy.is_shutdown():
        pass
