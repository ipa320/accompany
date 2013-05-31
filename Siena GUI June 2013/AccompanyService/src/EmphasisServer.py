#!/usr/bin/env python
import roslib;
import roslib.packages
roslib.load_manifest('AccompanyService')
import rospy;
import sys
import os

from AccompanyService.srv import *

import dynamic_reconfigure.client
DEFAULT_MAX_VEL=0.5
DEFAULT_MIN_VEL=0.15
MAX_VEL=0.7
MIN_VEL=0.09
client = dynamic_reconfigure.client.Client("/move_base/DWAPlannerROS")

def handle_emphasis_request(req):

	#print "min vel: %s"%min_vel
	#print "max_vel: %s"%max_vel
	val=float(req.action)
	#print "value:   %f"%val
	if (val<min_vel):
		val=min_vel
	elif (val>max_vel):
		val=max_vel
	print "Squeeze value: %s <--> Robot speed value: %f" % (req.action,val)	

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
	global min_vel
	global max_vel
	min_vel=DEFAULT_MIN_VEL
	max_vel=DEFAULT_MAX_VEL
	path = roslib.packages.get_pkg_dir('AccompanyService')+"/config/emphasis.config";
	print "config path: "+path
	f= open(path)
    	lines = f.readlines()
    	for line in lines:
		values = line.split()
		if values[0]=="min_speed":
			min_vel=float(values[1])
		elif values[0]=="max_speed":
			max_vel=float(values[1])
		else:
			print "error, unreadable line in configuration file"
			break
	f.close()

	# security check to avoid too high/low values to be setted
	if (min_vel<MIN_VEL):
		min_vel=MIN_VEL

	if (max_vel>MAX_VEL):
		max_vel= MAX_VEL

	emphasis_server()

