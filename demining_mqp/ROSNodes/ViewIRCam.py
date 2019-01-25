#!/usr/bin/env python
import rospy
import cv2
from demining_mqp.msg import*

class IRCamView:
    def __init__(self):
        self._processedImage = rospy.Subscriber('/ProcessedImage', image3, self.showimage, queue_size=5)

    def showimage(self, data):
        resizedImage = cv2.resize(data, (800, 600))
        cv2.imshow('image', resizedImage)
        cv2.waitkey(1)


if __name__ == '__main__':
    rospy.init_node('IRCamView')
    IRCamV = IRCamView()
    rospy.sleep(1)

    while not rospy.is_shutdown():
        pass
