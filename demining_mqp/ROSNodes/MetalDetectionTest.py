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
        self._sendSliderPos = rospy.Subscriber('/sliderPos', sliderposition, self.saveSliderData, queue_size=1)  # need to make message type
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
        self.FirstReading = True
        self.MineFirstDetected1 = -1
        self.MineLastDetected1 = -1
        self.MineFirstDetected2 = -1
        self.MineLastDetected2 = -1
        self.NewMineDetected1 = True
        self.NewMineDetected2 = True
        self.MineFirstDetected = -1
        self.MineLastDetected = -1
        self.sliderCurPos = 0
        self.sliderCurDir = 0
        self.didsliderchangedirection = False
        self.SensorArmStatus = 0
    def scanForMine(self):
        if self.FirstReading:
            print("Scanning for first Reading")
            if self.NewMineDetected1 and self.MineDetected:
                self.NewMineDetected1 = False
                self.MineFirstDetected1 = self.sliderCurPos
            elif self.NewMineDetected1 is False and self.MineDetected:
                self.MineLastDetected1 = self.sliderCurPos
                if self.didsliderchangedirection:
                    self.MineFirstDetected = self.MineFirstDetected1
                    self.MineLastDetected = self.MineLastDetected1
                    self.markLandmine()
                    self.NewMineDetected1 = True
            elif self.NewMineDetected1 is False and self.MineDetected is False:
                self.FirstReading = False
                #self.markLandmine()
                self.NewMineDetected1 = True
        else:
            print("Scanning for second Reading")
            if self.NewMineDetected2 and self.MineDetected:
                self.NewMineDetected2 = False
                self.MineFirstDetected2 = self.sliderCurPos
            elif self.NewMineDetected2 is False and self.MineDetected:
                self.MineLastDetected2 = self.sliderCurPos
                if self.didsliderchangedirection:
                    self.MineFirstDetected = self.MineLastDetected1
                    self.MineLastDetected = self.MineFirstDetected2
                    self.markLandmine()
                    self.NewMineDetected2 = True
                    self.FirstReading = True
            elif self.NewMineDetected2 is False and self.MineDetected is False:
                self.FirstReading = True
                self.MineFirstDetected = self.MineLastDetected1
                self.MineLastDetected = self.MineFirstDetected2
                self.markLandmine()
                self.NewMineDetected2 = True
            else:
                if np.abs(self.MineLastDetected1-self.sliderCurPos) > 400:
                    print("Second Reading not found")
                    self.MineFirstDetected = self.MineFirstDetected1
                    self.MineLastDetected = self.MineLastDetected1
                    self.markLandmine()
                    self.FirstReading = True

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
            NeededPos = (self.MineLastDetected+self.MineFirstDetected)/2
        print(NeededPos)
        scommand = slidercommand()
        scommand.motorstep = NeededPos
        scommand.scanFreely = False
        self._sendSliderMsg.publish(scommand)
        rospy.sleep(1)
        while self.sliderCurPos != NeededPos:
            rospy.sleep(.5)
        self._sendSAStatus.publish(self.MsgExtendPaint)
        while self.SensorArmStatus != self.StsSprayingPaint:
            rospy.sleep(.5)
        rospy.sleep(1)#simulating robot driving back
        self._sendSAStatus.publish(self.MsgRetractPaint)
        while self.SensorArmStatus != self.StsRunning:
            rospy.sleep(.5)
        scommand = slidercommand()
        scommand.motorstep = self.MineLastDetected
        scommand.scanFreely = False
        self._sendSliderMsg.publish(scommand)
        while self.sliderCurPos != self.MineLastDetected:
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
