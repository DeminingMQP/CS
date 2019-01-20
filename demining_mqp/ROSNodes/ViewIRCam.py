#! /usr/bin/env python
import rospy
import numpy as np
import cv2

#this node should display the images recieved from the CompVision Node on Nick's Laptop

class IRCamView:
    def __init__(self):
        self._processedImage = rospy.Subscriber('/ProcessedImage', image, self.showimage, queue_size=5)

    def showimage(self, data):
        resizedImage = cv2.resize(data, (800,600))
        cv2.imshow('image', (resizedImage))
        cv2.waitkey(1)


if __name__ == '__main__':
    rospy.init_node('IRCamView')
    IRCam = IRCamView()
    rospy.sleep(1)

    while not rospy.is_shutdown():
        pass
