#!/usr/bin/env python
import roslib; 
roslib.load_manifest('AccompanyService')
import sys

import rospy;
from AccompanyService.srv import *

def add_two_ints_client(action,uid):
    rospy.wait_for_service('ActionsService')
    try:
        add_two_ints = rospy.ServiceProxy('ActionsService', AccompanyAction)
        resp1 = add_two_ints(action,1)
        return resp1.result
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [action]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        action = sys.argv[1]
    else:
        print usage()
        sys.exit(1)
    print "Requesting %s..."%(action)
    print "result [%s] = %s"%(action, add_two_ints_client(action,1))

