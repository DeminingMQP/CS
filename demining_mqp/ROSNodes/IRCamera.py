#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from pylepton import Lepton
import time
from demining_mqp.msg import *
from rospy.numpy_msg import *



class IRCam:
    def __init__(self):
        self._receivedImage = rospy.Publisher('/RawIRImage', image, queue_size=5)
        self.last_nr = None

    def capture(self):
        with Lepton("/dev/spidev0.0") as l:
            lepton_buf = np.zeros((60, 80, 1), dtype=np.uint16)
            last_nr = 0
            _, nr = l.capture(lepton_buf)
            if nr != self.last_nr:
                self.last_nr = nr
                cv2.normalize(lepton_buf, lepton_buf, 0, 65535, cv2.NORM_MINMAX)
                np.right_shift(lepton_buf, 8, lepton_buf)
                lepton_buf = np.uint8(lepton_buf)
                finalImage = image
                for x in lepton_buf:
                    for y in x:
                        for z in y:
                            finalImage.data.append(z)
                print(finalImage)
                self._receivedImage.publish(finalImage)


if __name__ == '__main__':
    rospy.init_node('IRCam')
    IRCam = IRCam()
    rospy.sleep(1)

    while not rospy.is_shutdown():
        time.sleep(.1)
        IRCam.capture()