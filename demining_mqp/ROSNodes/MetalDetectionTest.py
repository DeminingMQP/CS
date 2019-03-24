#!/usr/bin/env python
from std_msgs.msg import *
import rospy
import numpy as np
from demining_mqp.msg import *

class TestMD:
    def __init__(self):
        self._sendSAStatus = rospy.Publisher('/NavCommand', std_msgs.msg.Int8, queue_size=5)# send to SensorPlatform Control
        self._sendSliderMsg = rospy.Publisher('sliderCommand', slidercommand, queue_size=5)
        self._retrieveMD = rospy.Subscriber('/LandmineDetected', std_msgs.msg.Int8, self.saveMineDetectInfo, queue_size=5)
        self._sendSliderPos = rospy.Subscriber('/sliderPos', sliderposition, self.saveMineDetectInfo, queue_size=1)  # need to make message type
        self._retrieveStatus = rospy.Subscriber('/SensorArmStatus', std_msgs.msg.Int8, self.SaveSensorArmStatus, queue_size=5)  # send to nav system
        self.MsgStart = 1
        self.MsgZeroMetalDetector = 2
        self.MsgHomeOrientation = 3
        self.MsgMarkLandmine = 4
        self.MsgStop = 5
        self.MsgExtendPaint = 6
        self.MsgRetractPaint = 7
        self.StsRunning = 0
        self.StsStopped = 1
        self.StsZeroingMetalMD = 2
        self.StsErrorZeroMD = 3
        self.StsHomingCoil = 4
        self.StsErrorHomingCoil = 5
        self.StsErrorMotorStall = 6
        self.StsMarkingLandmine = 7
        self.StsCommandUnknown = 8
        self.StsSprayingPaint = 9
        self.StsExtendingPaint = 10
        self.StsRetractingPaint = 11
        self.StsGeneralError = 12
        self.MineDetected = False
        self.MineFirstDetected = -1
        self.MineLastDetected = -1
        self.NewMineDetected = True
        self.sliderCurPos = 0
        self.sliderCurDir = 0
        self.didsliderchangedirection = False
        self.SensorArmStatus = 0
    def scanForMine(self):
        if self.NewMineDetected and self.MineDetected:
            self.NewMineDetected = False
            self.MineFirstDetected = self.sliderCurPos
        elif self.NewMineDetected is False and self.MineDetected:
            self.MineLastDetected = self.sliderCurPos
            if self.didsliderchangedirection:
                self.markLandmine()
                self.NewMineDetected = True
        elif self.NewMineDetected is False and self.MineDetected is False:
            self.markLandmine()
            self.NewMineDetected = True
    def saveSliderData(self, data):
        self.sliderCurPos = data.motorstep
        if data.direction is not self.sliderCurDir:
            self.didsliderchangedirection = True
        else:
            self.didsliderchangedirection = False
        self.sliderCurDir = data.direction

    def saveMineDetectInfo(self, data):
        if data.data is not 0:
            self.MineDetected = True
        else:
            self.MineDetected = False
    def SaveSensorArmStatus(self, data):
        self.SensorArmStatus = data.data
    def markLandmine(self):
        NeededPos = 0
        if self.didsliderchangedirection:
            NeededPos = self.MineLastDetected
        else:
            NeededPos = (self.MineLastDetected-self.MineFirstDetected)/2
        scommand = slidercommand()
        scommand.motorstep = NeededPos
        scommand.scanFreely = False
        self._sendSliderMsg.publish(scommand)
        while self.sliderCurPos is not NeededPos:
            rospy.sleep(.5)
        self._sendSAStatus.publish(self.MsgExtendPaint)
        while self.SensorArmStatus is not self.StsSprayingPaint:
            rospy.sleep(.5)
        rospy.sleep(1)#simulating robot driving back
        self._sendSAStatus.publish(self.MsgRetractPaint)
        while self.SensorArmStatus is not self.StsRunning:
            rospy.sleep(.5)
        scommand = slidercommand()
        scommand.motorstep = -1
        scommand.scanFreely = True
        self._sendSliderMsg.publish(scommand)
if __name__ == '__main__':
    rospy.init_node('MDTest')
    test = TestMD()
    rospy.sleep(1)
    while not rospy.is_shutdown():
        test.scanForMine()
