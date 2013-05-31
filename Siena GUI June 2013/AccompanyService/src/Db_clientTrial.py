#!/usr/bin/env python
import roslib; 
roslib.load_manifest('AccompanyService')
import sys

import rospy;
from AccompanyService.srv import *

def add_two_ints_client(action,param):
    rospy.wait_for_service('DatabaseConnectorService')
    try:
        add_two_ints = rospy.ServiceProxy('DatabaseConnectorService', db_msg)
	act=int(action)
        resp1 = add_two_ints(act,param)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [action]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        action = sys.argv[1]
 	print "Requesting %s..."%(action)
	p=""
	resp =add_two_ints_client(action,p)
    	print "result [%s] = %s code %s"%(action,resp.answer ,resp.code)
    elif len(sys.argv) == 3:
	action= sys.argv[1]
	param=  sys.argv[2]
	
	print "Requesting %s... [%s]"%(action,param)
	resp =add_two_ints_client(action,param)
    	print "result [%s[%s]] = %s, code %s"%(action,param, resp.answer,resp.code)
    else:
        print usage()
        sys.exit(1)
   

