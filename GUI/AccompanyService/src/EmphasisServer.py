#!/usr/bin/env python
import roslib;
roslib.load_manifest('AccompanyService')
import rospy;
import sys
import os

from AccompanyService.srv import *

import dynamic_reconfigure.client

MIN_VEL=0.1
MAX_VEL=0.75
client = dynamic_reconfigure.client.Client("/move_base/DWAPlannerROS")

def handle_emphasis_request(req):

	val=float(req.action)
	if (val<MIN_VEL):
		val=MIN_VEL
	elif (val>MAX_VEL):
		val=MAX_VEL
	print "Squeeze value: %s <--> Robot speed value: %s" % (req.action,val)	

	params={'max_rot_vel':val, 'max_trans_vel':val,'max_vel_x':val,'max_vel_y':val,'min_vel_x':-val,'min_vel_y':-val}
	config=client.update_configuration(params);
	return 1

def emphasis_server():
	rospy.init_node('EmphasisService')
	s = rospy.Service('EmphasisService',AccompanyAction,handle_emphasis_request)
	print "Accompany Emphasis service running..."
	#client = dynamic_reconfigure.client.Client("/move_base/DWAPlannerROS")
	rospy.spin()

if __name__=="__main__":
	emphasis_server()

