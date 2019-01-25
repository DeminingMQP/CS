#!/usr/bin/env python
import rospy
import cv2
from demining_mqp.msg import*
from rospy.numpy_msg import *
import numpy as np

class IRCamView:
    def __init__(self):
        self._processedImage = rospy.Subscriber('/ProcessedImage', image, self.showimage, queue_size=5)

    def showimage(self, data):
        Image = data
        reconstruct = np.zeros((60, 80, 1), dtype=np.uint8)

        index = 0
        for x in range(0, 60):
            for y in range(0, 80):
                reconstruct[x][y][0] = ord(Image.data[index])
                index = index + 1
        print "Reconstructed image"
        resizedImage = cv2.resize(reconstruct, (800, 600))
        cv2.imshow('image', resizedImage)
        cv2.waitkey(1)


if __name__ == '__main__':
    rospy.init_node('IRCamView')
    IRCamV = IRCamView()
    rospy.sleep(1)

    while not rospy.is_shutdown():
        pass
