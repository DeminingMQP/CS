import rospy
from geometry_msgs.msg import Point
import serial
import sys
import select
import time


def main():
    pub = rospy.Publisher('/dgps_location', Point, queue_size=10)
    rospy.init_node('dgps', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    loc_msg = Point()
    port = serial.Serial("/dev/ttyACM0", baudrate=115200, timeout=3.0)
    minLat=200
    minLon=200
    maxLat=0
    maxLon=0
    waitTime =0
    while not rospy.is_shutdown():
        try:
            rcv = port.readline()
            #if True:
            if rcv[0:6] == "$GNGLL":
                print(repr(rcv))
                #print(rcv.decode('utf-8'))
                rawLatitude = rcv[7:17]
                rawLongitude = rcv[20:31]
                loc_msg.x = int(rawLatitude[0:2]) + float(rawLatitude[2:])/60
                loc_msg.y = int(rawLongitude[0:3]) + float(rawLongitude[3:])/60   
        except KeyboardInterrupt:
            exit(1)
        except Exception as e:
            print("error", e)
        #rospy.loginfo(loc_msg)
        if time.time() >= waitTime+1:
            pub.publish(loc_msg)
            waitTime=time.time()
        #rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
