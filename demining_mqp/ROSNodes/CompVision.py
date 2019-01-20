#! /usr/bin/env python
import rospy
import numpy as np
import cv2
from std_msgs.msg import String, Bool

class CompVisionNode:
    def __init__(self):
        self._recievedImage = rospy.Subscriber('/RawIRImage', Image, queue_size = 5)
        self._processedImage = rospy.Publisher('/ProcessedImage', Image, queue_size  =5)
        self._LandmineDetected = rospy.Publisher('/IRLandmineDet', Bool)
    def SearchForMine(self, data):


        #add ROS code to recieve image message from IRCamera Node
        #must be grayscale image which it should be from the camera
        gray = 1 #make this equal to the image from the camera
        output = gray.copy()
        circles = cv2.HoughCircles(gray, cv2.cv.CV_HOUGH_GRADIENT, 1.2, 100)#last arg is pixels
        if circles is not None:
            circles = np.round(circles[0, :]). astype("int")
            for(x,y,r) in circles:
                cv2.circle(output, (x,y), r, (0, 255, 0), 4)
                cv2.rectangle(output, (x-5, y-5), (x+5, y+5), (0, 128, 255), -1)
            #send image to laptop for viewing with ROS message. Should show circles if any
if __name__ == '__main__':
    rospy.init_node('CompVision')
    CompVis = CompVisionNode()
    rospy.sleep(1)

    while not rospy.is_shutdown():
        pass