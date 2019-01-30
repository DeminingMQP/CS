#!/usr/bin/env python

from demining_mqp.srv import *
import rospy

def handle_batteryLife(req):
    print "Returning battery life"
    print(type(req.oldBatteryLife))
    if(req.x == req.chargeLoc and req.y == req.chargeLoc):
	return (req.maxBattery)
    else:
        return (req.oldBatteryLife - 1)

def battery_life_server():
    rospy.init_node('battery_life_server')
    s = rospy.Service('battery_life', batteryLifeService, handle_batteryLife)
    print("Ready to say battery levels")    
    rospy.spin()

if __name__ == "__main__":
    battery_life_server()
