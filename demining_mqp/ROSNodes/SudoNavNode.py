#!/usr/bin/env python
from std_msgs.msg import Bool
import rospy
import numpy as np
from demining_mqp.msg import *
import std_msgs

class sudonavnode:
    def __init__(self):
        self._StartSweep = rospy.Publisher('/StartSweep', std_msgs.msg.Int8, queue_size=1)
        self._ContinueSweep = rospy.Publisher('/ContinueSweep', std_msgs.msg.Int8, queue_size=1)
        self._StopSweep = rospy.Subscriber('/StopSweep', std_msgs.msg.Int8, self.SweepStopCallback, queue_size=1)
        self._MineFound = rospy.Subscriber('/MineFound', std_msgs.msg.Int8, self.LandmineFoundCallback, queue_size=1)
        self.SweepOccuring = False
        self.LandmineFound = False
    def Navigate(self):
        if self.SweepOccuring == False:
            rospy.sleep(2)
            self._StartSweep.publish(1)
            self.SweepOccuring = True
        elif self.LandmineFound:
            rospy.sleep(1)
            self._ContinueSweep.publish(1)
            self.LandmineFound = False
        else:
            rospy.sleep(1)

    def SweepStopCallback(self, data):
        self.SweepOccuring = False

    def LandmineFoundCallback(self, data):
        self.LandmineFound = True

if __name__ == '__main__':
    rospy.init_node('SudoNav')
    nav = sudonavnode()
    rospy.sleep(1)

    while not rospy.is_shutdown():
        nav.Navigate()
        print("Running")
