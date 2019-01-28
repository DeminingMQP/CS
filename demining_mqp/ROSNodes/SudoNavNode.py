#!/usr/bin/env python
from std_msgs.msg import Bool
import rospy
import numpy as np
from demining_mqp.srv import *
import std_msgs

class sudonavnode:
    def __init__(self):
        self._sendSAStatus = rospy.Publisher('/NavCommand', std_msgs.msg.Int8, queue_size=5)# send to SensorPlatform Control

    def sendmessage(self, data):
        self._sendSAStatus.publish(data)
if __name__ == '__main__':
    rospy.init_node('SudoNav')
    nav = sudonavnode()
    rospy.sleep(1)

    while not rospy.is_shutdown():
        print "Stopping"
        nav.sendmessage(1)
        rospy.sleep(5)
        print "Starting"
        nav.sendmessage(5)
        rospy.sleep(5)