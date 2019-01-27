#!/usr/bin/env python
import rospy
from demining_mqp.srv import*


class iicSemaphore:
    def __init__(self):
        self.curNodeID = 0
        self.semcount = 1

    def semServer(self):
        rospy.Service('IICAccess', IICSemSrv, self.handle_request)
        rospy.spin()

    def handle_request(self, requested):
        if requested.checkinout == 0: #0 for check out
            if self.semcount == 1:
                self.curNodeID = requested.id
                self.semcount = 0
                return IICSemSrvResponse(True)
            else:
                if requested.id == self.curNodeID:
                    return IICSemSrvResponse(True)
                else:
                    return IICSemSrvResponse(False)
        else:
            if requested.id == self.curNodeID:
                self.semcount = 1
                return IICSemSrvResponse(True)
            else:
                return IICSemSrvResponse(False)

if __name__ == '__main__':
    rospy.init_node('IICSem')
    Server = iicSemaphore()
    rospy.sleep(1)
    Server.semServer()

    while not rospy.is_shutdown():
        pass
