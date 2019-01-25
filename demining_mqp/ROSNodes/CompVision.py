#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from std_msgs.msg import Bool

from demining_mqp.msg import*
from rospy.numpy_msg import *


class CompVisionNode:
    def __init__(self):
        self._receivedImage = rospy.Subscriber('/RawIRImage', image, self.searchformine, queue_size=5)
        self._processedImage = rospy.Publisher('/ProcessedImage', image, queue_size=5)
        self._LandmineDetected = rospy.Publisher('/IRLandmineDet', Bool, queue_size=1)

    def searchformine(self, data):

        # must be gray-scale image which it should be from the camera
        gray = data.data  # make this equal to the image from the camera
        reconstruct = np.zeros((60, 80, 1), dtype=np.uint8)
        index = 0
        for x in range (0,80):
            for y in range(0,60):
                reconstruct[0][x][y]= gray[index]
                index = index +1

        output = reconstruct.copy()
        circles = cv2.HoughCircles(reconstruct, cv2.cv.CV_HOUGH_GRADIENT, 1.2, 100)  # last arg is pixels
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for (x, y, r) in circles:
                cv2.circle(output, (x, y), r, (0, 255, 0), 4)
                cv2.rectangle(output, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
            output = np.uint8(output)
            finalImage = []
            for x in output:
                for y in x:
                    for z in y:
                        finalImage.append(z)
            self._processedImage.publish(finalImage)#might need different message type because i think this will have color
            self._LandmineDetected.publish(True)
        else:
            self._LandmineDetected.publish(False)


if __name__ == '__main__':
    rospy.init_node('CompVision')
    CompVis = CompVisionNode()
    rospy.sleep(1)

    while not rospy.is_shutdown():
        pass
