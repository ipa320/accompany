#!/usr/bin/env python
import roslib;
import roslib.packages
roslib.load_manifest('accompany_siena_squeeze_service')
import rospy;
import sys
import os
import math

from accompany_siena_squeeze_service.srv import *
import MySQLdb


from simple_script_server import *
sss = simple_script_server()

import dynamic_reconfigure.client
DEFAULT_MAX_VEL=0.5
DEFAULT_MIN_VEL=0.15
DEFAULT_DEFAULT_SPEED=0.25
MAX_VEL=0.51
MIN_VEL=0.05
client = dynamic_reconfigure.client.Client("/move_base/DWAPlannerROS")
db = MySQLdb.connect(host="10.0.1.207",
					user="accompanyUser",
					passwd="accompany",
					db="AccompanyTroyes")
cur = db.cursor()


def handle_emphasis_request(req):

	cur.execute("""UPDATE Sensors SET value = 'True' WHERE sensorId = 592""")
	#print "min vel: %s"%min_vel
	#print "max_vel: %s"%max_vel
	val=float(req.action)/2.0
	#print "value:   %f"%val
	if (val<0.01):
		print "reset..."
		val=default_speed
	if (val<min_vel):
		val=min_vel
	elif (val>max_vel):
		val=max_vel
	print "Squeeze value: %s <--> Robot speed value: %f" % (req.action,val)	

	val2 = math.sqrt(2)*val
	params={'max_rot_vel':val2, 'max_trans_vel':val2,'max_vel_x':val,'max_vel_y':val,'min_vel_x':-val,'min_vel_y':-val}
	print "update config"
	config=client.update_configuration(params);
	
	#little modification
	print "move now"
	#sss.move("base",[2.750,-0.449,2.325],False)
	#sss.move("base",[1.50,-1.8,1.711],False)
	#sss.set_light("flashing yellow")
	#sss.move("base",[2.731,-0.849,1.795],False)
	
	
	return 1

def emphasis_server():
	rospy.init_node('EmphasisService')
	s = rospy.Service('EmphasisService',AccompanyAction,handle_emphasis_request)
	print "Accompany Emphasis service running..."
	client = dynamic_reconfigure.client.Client("/move_base/DWAPlannerROS")
	rospy.spin()

if __name__=="__main__":	
	global min_vel
	global max_vel
	global default_speed
	min_vel=DEFAULT_MIN_VEL
	max_vel=DEFAULT_MAX_VEL
	default_speed=DEFAULT_DEFAULT_SPEED
	path = roslib.packages.get_pkg_dir('accompany_siena_squeeze_service')+"/config/emphasis.config";
	print "config path: "+path
	f= open(path)
    	lines = f.readlines()
    	for line in lines:
		values = line.split()
		if values[0]=="min_speed":
			min_vel=float(values[1])
		elif values[0]=="max_speed":
			max_vel=float(values[1])
		elif values[0]=="default_speed":
			default_speed=float(values[1])
		else:
			print "error, unreadable line in configuration file"
			break
	f.close()

	# security check to avoid too high/low values to be setted
	if (min_vel<MIN_VEL):
		min_vel=MIN_VEL

	if (max_vel>MAX_VEL):
		max_vel= MAX_VEL
		
	print "min_vel", min_vel, "   max_vel", max_vel, "    default_vel", default_speed

	emphasis_server()

