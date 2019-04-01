#!/usr/bin/env python
from std_msgs.msg import Bool
import rospy
import numpy as np
from demining_mqp.msg import *
import std_msgs

class sudonavnode:
    def __init__(self):
        self._sendSAStatus = rospy.Publisher('/NavCommand', std_msgs.msg.Int8, queue_size=5)# send to SensorPlatform Control
        self._sendSliderMsg = rospy.Publisher('sliderCommand', slidercommand, queue_size=5)
        self.MsgStart = 1
        self.MsgZeroMetalDetector = 2
        self.MsgHomeOrientation = 3
        self.MsgMarkLandmine = 4
        self.MsgStop = 5
        self.MsgExtendPaint = 6
        self.MsgRetractPaint = 7
    def sendmessage(self, data, data2):
        self._sendSAStatus.publish(data)
        scommand = slidercommand()
        scommand.motorstep=0
        scommand.scanFreely = data2
        self._sendSliderMsg.publish(scommand)
if __name__ == '__main__':
    rospy.init_node('SudoNav')
    nav = sudonavnode()
    rospy.sleep(1)

    while not rospy.is_shutdown():
        rospy.sleep(20)
        print ("Stopping")
        nav.sendmessage(nav.MsgExtendPaint, False)
        rospy.sleep(20)
        print ("Starting")
        nav.sendmessage(nav.MsgRetractPaint, True)
        rospy.sleep(10)
