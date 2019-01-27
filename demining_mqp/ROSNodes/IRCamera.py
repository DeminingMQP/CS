#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from pylepton import Lepton
import time
from demining_mqp.msg import *
from demining_mqp.srv import *
from std_msgs import *



class IRCam:
    def __init__(self):
        self._receivedImage = rospy.Publisher('/RawIRImage', image, queue_size=5)
        self.last_nr = None
        self.iicSemId = 246
        print "started"

    def capture(self):
        with Lepton("/dev/spidev0.0") as l:
            lepton_buf = np.zeros((60, 80, 1), dtype=np.uint16)
            last_nr = 0
            #print "Getting Image"
            _, nr = l.capture(lepton_buf)
            print "got image"
            if nr != self.last_nr:
                self.last_nr = nr
                cv2.normalize(lepton_buf, lepton_buf, 0, 65535, cv2.NORM_MINMAX)
                np.right_shift(lepton_buf, 8, lepton_buf)
                lepton_buf = np.uint8(lepton_buf)
                finalImage = []

                for x in lepton_buf:
                    for y in x:
                        for z in y:
                            finalImage.append(z)

                self._receivedImage.publish(finalImage)
    def getIICSem(self):
        rospy.wait_for_service('IICAccess')
        response = 0
        try:
            getsem = rospy.ServiceProxy('IICAccess', IICSemSrv)
            response = getsem(self.iicSemId, 0)
        except rospy.ServiceException, e:
            print "oops, something went wrong  with the IIC sem"
        if response.access is True:
            return True
        else:
            return False
    def returnIICSem(self):
        rospy.wait_for_service('IICAccess')
        response = 0
        try:
            returnsem = rospy.ServiceProxy('IICAccess', IICSemSrv)
            response = returnsem(self.iicSemId, 1)
        except rospy.ServiceException, e:
            print "oops, something went wrong  with the IIC sem"
        if response.access is True:
            return True
        else:
            return False

if __name__ == '__main__':
    rospy.init_node('IRCam')
    IRCam = IRCam()
    rospy.sleep(1)

    while not rospy.is_shutdown():
        time.sleep(.1)
        while IRCam.getIICSem() is False:
            rospy.sleep(0.01)
        print("Calling Capture")
        IRCam.capture()
        if IRCam.returnIICSem() is False:
            print "For some reason the IIC sem could not be returned"
        else:
            pass
