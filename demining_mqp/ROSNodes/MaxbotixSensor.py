#!/usr/bin/env python
import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import *
from sensor_msgs.msg import LaserScan
from demining_mqp.msg import*
import time, math


class maxbotix:
    def __init__(self):
        self._sendSensorData = rospy.Publisher('/Maxbotix', LaserScan, queue_size=5)  # send to nav system
        self._getsliderpos = rospy.Subscriber('/sliderPos', sliderposition, self.sliderCallback, queue_size=1)
        self.risingPin = 22
        self.fallingPin = 26
        GPIO.setmode(GPIO.BOARD)

        self.riseCallback = GPIO.add_event_detect(self.risingPin, GPIO.RISING, self.risinghandle())
        self.fallCallback = GPIO.add_event_detect(self.fallingPin, GPIO.FALLING, self.fallinghandle())
        self.pulseStart = 0
        self.pulseEnd = 0
        self.sliderpos = 0
        self.measurementCounter = 0
        self.savedMeasurements = []
        self.lastSliderDirection = 2
        self.startScanning = False

        self.distanceSliderFromOrigin = 0.3 #meters
    def sliderCallback(self, data):
        if self.startScanning is False:#this waits for the slider to reach extreme
            if self.lastSliderDirection is 2:
                self.lastSliderDirection = data.direction
            else:
                if self.lastSliderDirection is not data.direction:
                    self.startScanning = True
        else:
            if self.lastSliderDirection is not data.direction:
                lsavedData = self.savedMeasurements
                self.savedMeasurements = []
                self.measurementCounter = 0
                lsavedCount = self.measurementCounter
                sum = 0
                ranges = []
                for x in self.savedMeasurements:
                    sum = sum +x[0]
                    ranges.append(x[1])
                avgAngle = sum/lsavedCount

                data.sort(lsavedData)#should sort by angles
                scan = LaserScan()
                scan.header.frame_id = 1#not sure how to get the appropriate frame. Want the robot frame
                scan.angle_increment = avgAngle
                scan.angle_min = lsavedData[0][0]
                scan.angle_max = lsavedData[lsavedCount][0]
                scan.range_min = 0
                scan.range_max = 5
                scan.time_increment = 0.01 #find actual time
                scan.ranges = ranges
                self._sendSensorData.publish(scan)


        self.sliderpos = data.motorstep
        self.lastSliderDirection = data.direction

    def risinghandle(self):
        self.pulseStart = time.time()


    def fallinghandle(self):
        self.pulseEnd = time.time()
        xm = self.sliderpos*0.43 - 0.2 #change to number to convert steps to distance and half max steps
        ym = (self.pulseStart-self.pulseEnd)/1000 + self.distanceSliderFromOrigin
        dist = math.sqrt(math.pow(xm, xm)+math.pow(ym, ym))
        angle = math.tan(xm/ym)
        self.savedMeasurements.append((angle, dist))
        self.measurementCounter = self.measurementCounter +1


if __name__ == '__main__':
    rospy.init_node('MaxbotixSensor')
    Sensor = maxbotix()
    rospy.sleep(1)

    while not rospy.is_shutdown():
        pass
