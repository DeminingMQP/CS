#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from demining_mqp.msg import mapsAPI
from demining_mqp.msg import array1d

def talker():
    pub = rospy.Publisher('maps', mapsAPI, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = array1d()
	hello_str.array = {0}
	hello_arr = mapsAPI()
	hello_arr.grid = {hello_str}
        rospy.loginfo(hello_arr)
        pub.publish(hello_arr)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
