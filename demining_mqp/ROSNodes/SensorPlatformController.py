#!/usr/bin/env python
import rospy
import smbus
from std_msgs.msg import Bool
import numpy as np
from demining_mqp.srv import *
import std_msgs
class sensorplatcontrol:
    def __init__(self):
        self._sendSAStatus = rospy.Publisher('/SensorArmStatus', std_msgs.msg.Int8, queue_size=5)# send to nav system
        self._receiveCommand = rospy.Subscriber('/NavCommand', std_msgs.msg.Int8, self.handleCommand, queue_size=1 )
        self.bus = smbus.SMBus(1)
        self.addressUno = 0x08
        self.addressMega = 0x04
        self.MsgStart = 1
        self.MsgZeroMetalDetector = 2
        self.MsgHomeOrientation = 3
        self.MsgMarkLandmine = 4
        self.MsgStop = 5
        self.StsRunning = 0
        self.StsStopped = 1
        self.StsZeroingMetalMD = 2
        self.StsErrorZeroMD = 3
        self.StsHomingCoil = 4
        self.StsErrorHomingCoil = 5
        self.StsErrorMotorStall = 6
        self.StsMarkingLandmine = 7
        self.StsCommandUnknown = 8
        self.StsGeneralError = 9
        self.handleID = 568
        self.checkID = 297

    def sendMessage(self, msg, addr):
        self.bus.write_byte(addr, msg)

    def retrieveStatus(self, addr):
        status = self.bus.read_byte(addr)
        return status

    def handleCommand(self, data):

        while self.getIICSem(self.handleID) is False:
            rospy.sleep(.01)
        print "Got IICSem"
        if data.data is self.MsgStart:
            self.sendMessage(self.MsgStart, self.addressUno)
            self.sendMessage(self.MsgStart, self.addressMega)
            #rospy.sleep(1)
            #status1 = self.retrieveStatus(self.addressUno)
            #status2 = self.retrieveStatus(self.addressMega)
            #print(status1)
            #print(status2)
            #if status1 is self.StsRunning and status2 is self.StsRunning:
             #   self._sendSAStatus.publish(self.StsRunning)
            #else:
             #   self._sendSAStatus.publish(self.StsGeneralError)
        elif data.data is self.MsgStop:
            self.sendMessage(self.MsgStop, self.addressUno)
            self.sendMessage(self.MsgStop, self.addressMega)
           # rospy.sleep(1)
           # status1 = self.retrieveStatus(self.addressUno)
           # status2 = self.retrieveStatus(self.addressMega)
           # print(status1)
           # print(status2)
           # if status1 is self.StsStopped and status2 is self.StsStopped:
           #     self._sendSAStatus.publish(self.StsStopped)
           # else:
           #     self._sendSAStatus.publish(self.StsGeneralError)
        elif data.data is self.MsgZeroMetalDetector:
            self.ZeroMetalDetector()
        elif data.data is self.MsgHomeOrientation:
            self.HomeOrientation()
        elif data.data is self.MsgMarkLandmine:
            self.MarkLandmine()
        else:
            self._sendSAStatus.publish(self.StsCommandUnknown)

        if self.returnIICSem(self.handleID) is False:
            print "For some reason the IIC sem could not be returned"
        else:
            pass


    def HomeOrientation(self):
        self.sendMessage(self.MsgHomeOrientation, self.addressMega)
        rospy.sleep(1)
        homingbegun = False
        while homingbegun is False:
            if self.retrieveStatus(self.addressMega) is self.StsHomingCoil:
                homingbegun = True
            else:
                rospy.sleep(1)
        homingDone = False
        while homingDone is False:
            status = self.retrieveStatus(self.addressMega)
            if status is self.StsRunning:
                homingDone = True
                self._sendSAStatus.publish(self.StsRunning)
            elif status is self.StsErrorHomingCoil:
                self._sendSAStatus.publish(self.StsErrorHomingCoil)
            elif status is self.StsHomingCoil:
                self._sendSAStatus.publish(self.StsHomingCoil)
                rospy.sleep(1)
            else:
                self._sendSAStatus.publish(self.StsGeneralError)

    def ZeroMetalDetector(self):
        self.sendMessage(self.MsgZeroMetalDetector, self.addressUno)
        rospy.sleep(1)
        Zerobegun = False
        while Zerobegun is False:
            if self.retrieveStatus(self.addressUno) is self.StsZeroingMetalMD:
                Zerobegun = True
            else:
                rospy.sleep(1)
        ZeroDone = False
        while ZeroDone is False:
            status = self.retrieveStatus(self.addressUno)
            if status is self.StsRunning:
                ZeroDone = True
                self._sendSAStatus.publish(self.StsRunning)
            elif status is self.StsErrorZeroMD:
                self._sendSAStatus.publish(self.StsErrorZeroMD)
            elif status is self.StsZeroingMetalMD:
                self._sendSAStatus.publish(self.StsZeroingMetalMD)
                rospy.sleep(1)
            else:
                self._sendSAStatus.publish(self.StsGeneralError)

    def MarkLandmine(self):
        self.sendMessage(self.MsgMarkLandmine,self.addressMega)
        rospy.sleep(1)
        markingbegun = False
        while markingbegun is False:
            if self.retrieveStatus(self.addressMega) is self.StsMarkingLandmine:
                markingbegun = True
            else:
                rospy.sleep(1)
        markingDone = False
        while markingDone is False:
            status = self.retrieveStatus(self.addressMega)
            if status is self.StsRunning:
                markingDone = True
                self._sendSAStatus.publish(self.StsRunning)
            elif status is self.StsErrorMotorStall:
                self._sendSAStatus.publish(self.StsErrorMotorStall)
            elif status is self.StsMarkingLandmine:
                self._sendSAStatus.publish(self.StsMarkingLandmine)
                rospy.sleep(1)
            else:
                self._sendSAStatus.publish(self.StsGeneralError)
    def checkSensorArmStatus(self):
        status1 = self.retrieveStatus(self.addressUno)
        status2 = self.retrieveStatus(self.addressMega)
        print(status1)
        print(status2)
        if status1 is self.StsRunning and status2 is self.StsRunning:
            self._sendSAStatus.publish(self.StsRunning)
        elif status1 is self.StsStopped and status2 is self.StsStopped:
            self._sendSAStatus.publish(self.StsStopped)
        elif status2 is self.StsErrorMotorStall:
            self._sendSAStatus.publish(self.StsErrorMotorStall)
        else:
            self._sendSAStatus.publish(self.StsGeneralError)
        print "Got Status"
    def getIICSem(self, id):
        rospy.wait_for_service('IICAccess')
        response = 0
        try:
            getsem = rospy.ServiceProxy('IICAccess', IICSemSrv)
            response = getsem(id, 0)
        except rospy.ServiceException, e:
            print "oops, something went wrong  with the IIC sem"
        if response.access is True:
            return True
        else:
            return False
    def returnIICSem(self, id):
        rospy.wait_for_service('IICAccess')
        response = 0
        try:
            returnsem = rospy.ServiceProxy('IICAccess', IICSemSrv)
            response = returnsem(id, 1)
        except rospy.ServiceException, e:
            print "oops, something went wrong  with the IIC sem"
        if response.access is True:
            return True
        else:
            return False

if __name__ == '__main__':
    rospy.init_node('SensorPlatControl')
    SensorPlatformController = sensorplatcontrol()
    rospy.sleep(1)

    while not rospy.is_shutdown():
        while SensorPlatformController.getIICSem(SensorPlatformController.checkID) is False:
            rospy.sleep(.01)
        SensorPlatformController.checkSensorArmStatus()
        if SensorPlatformController.returnIICSem(SensorPlatformController.checkID) is False:
            print "For some reason the IIC sem could not be returned"
        else:
            pass
        rospy.sleep(5)