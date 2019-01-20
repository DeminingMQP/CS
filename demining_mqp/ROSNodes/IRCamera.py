#! /usr/bin/env python
import rospy
import numpy as np
import cv2
from pylepton import Lepton
import time


class IRCam:
    def __init__(self):
        self._receivedImage = rospy.Publisher('/RawIRImage', image, queue_size=5)

    def capture(self):
        with Lepton("/dev/spidev0.0") as l:
            lepton_buf = np.zeros((60, 80, 1), dtype=np.uint16)

            last_nr = 0

            while True:
                time.sleep(.1)
                _, nr = l.capture(lepton_buf)
                if nr == last_nr:
                 # no need to redo this frame
                    continue
                last_nr = nr
                cv2.normalize(lepton_buf, lepton_buf, 0, 65535, cv2.NORM_MINMAX)
                np.right_shift(lepton_buf, 8, lepton_buf)
                FinalImage = np.uint8(lepton_buf)

