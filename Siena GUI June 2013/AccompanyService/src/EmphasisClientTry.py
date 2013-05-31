#!/usr/bin/env python
import roslib; 
roslib.load_manifest('AccompanyService')
import sys

import rospy;
from AccompanyService.srv import *

def add_two_ints_client(action,param):
    rospy.wait_for_service('EmphasisService')
    try:
        add_two_ints = rospy.ServiceProxy('EmphasisService', AccompanyAction)
	#act=float(action)
        resp1 = add_two_ints(action,1)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [emphasis]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        action = sys.argv[1]
 	print "Requesting %s..."%(action)
	p=""
	resp =add_two_ints_client(action,p)
    	print "result [%s] = %s "%(action,resp.result)

    else:
        print usage()
        sys.exit(1)
