#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point

class baseListener:
    def __init__(self):
        self.mineLocations = []
        self._receiveMine = rospy.Subscriber('/mineLocation',Point, self.record, queue_size=1 )

	def record(self,data):
	    print data
        self.mineLocations.append(data)
	    
	    
if __name__ == '__main__':
    rospy.init_node('base', anonymous=True)
    base = baseListener()
    rospy.spin()

