#!/usr/bin/env python
import roslib;
roslib.load_manifest('AccompanyService')
import rospy;
import sys
import os

from AccompanyService.srv import *

import actionlib

from cob_script_server.msg import *
from simple_script_server import *

from cStringIO import StringIO
import re

sss=simple_script_server()

def str2bool(v):
	return v.lower() in ("yes", "true", "1")

def get_user_position_from_db(object_id):
	print "     ---- sending request to db [user: %s]..."% (object_id)
	rospy.wait_for_service('DatabaseConnectorService')
	try:
  		position= rospy.ServiceProxy('DatabaseConnectorService',db_msg)
		pos = position(10,str(object_id),0)
		return pos.answer
	except rospy.ServiceException, e:
		print "Service exception: ERROR	"		
		return "-1"

def get_object_position_from_db(object_id):
	print "     ---- sending request to db [object: %s]..."% (object_id)
	rospy.wait_for_service('DatabaseConnectorService')
	try:
  		position= rospy.ServiceProxy('DatabaseConnectorService',db_msg)
		pos = position(11,str(object_id),0)
		return pos.answer
	except rospy.ServiceException, e:
		print "Service exception: ERROR	"		
		return "-1"
	
	
#method that given an action associate replace the placeholders with correct positions
def filter_action(action,uid):
	#search=re.compile(r'@[a-z0-9_A-Z]+').search
	print "before search:"+action
	#if bool(search(action)==False):
	if action[0]=='@':
		print "false"
		action= action.replace("@","");
		if action == "user"or action== "User":
			# recover the user position from user id
			action=get_user_position_from_db(uid)
		else:
			# recover the object position directly from the object id 
			# we assume the object referred as @<object_id>
			action = get_object_position_from_db(action)
		
	return action

# Check if the command contains row postions or only params positions
def checkCommand(tokens):
	result=True
	search=re.compile(r'[^a-z0-9_A-Z/]+').search
	for s in tokens:
		if bool(search(s))==True:
			 result = False	
	return result

def do(action,uid):
	print "do: "+action
	print "--> do: "+action
	splitted = action.split()
	#for s in splitted:
	#	print s
	print "----"
	print splitted[0]
	print splitted[1]	
	splitted[1]= filter_action(splitted[1],uid)
	#print checkCommand(splitted)
	print "----"
	if checkCommand(splitted):
		if splitted[0] == "say":
			sss.say(splitted[1])
		elif splitted[0] == "move_base":
			sss.move("base",splitted[1])
		elif splitted[0] == "move_tray":
			sss.move("tray",splitted[1])
		elif splitted[0] == "sleep":
			sss.sleep(float(splitted[1]))
		elif splitted[0] == "move_arm":
			sss.move("arm",splitted[1],str2bool(splitted[2]))
		elif splitted[0] == "move_sdh":
			sss.move("sdh",splitted[1],str2bool(splitted[2]))
		elif splitted[0]== "move_head":
			sss.move("head",splitted[1])
		elif splitted[0]== "move_torso":
			sss.move("torso",splitted[1],str2bool(splitted[2]))
	else:
		if splitted[0] == "say":
			sss.say(eval(splitted[1].replace("_"," ")))
		elif splitted[0] == "move_base":
  			 print "length %s"% len(splitted) 
			 if (len(splitted)<3):
				sss.move("base",eval(splitted[1]))
			 else: 				
				splitted[1]=splitted[1].replace("[","")
				splitted[1]=splitted[1].replace("]","")
				splitted[2]=splitted[2].replace("[","")
				splitted[2]=splitted[2].replace("]","")
				print splitted[1]
				dest = splitted[1].split(',')
				subtr = splitted[2].split(',')	
				print "%f"%(float(dest[0])-float(subtr[0]))
				print "%f"%(float(dest[1])-float(subtr[1]))
				print "%f"%(float(dest[2])-float(subtr[2]))
				res="[%f"%(float(dest[0])-float(subtr[0]))
				print res
				res = res+",%f"%(float(dest[1])-float(subtr[1]))
				print res
				res = res +",%f]"%float(dest[2])		
				#splitted[1][0]="%s"%(float(splitted[1][0])-float(splitted[2][0]))
				#splitted[1][1]="%s"%(float(splitted[1][1])-float(splitted[2][1]))
				#splitted[1][2]="%s"%(float(splitted[1][2])-float(splitted[2][2]))
                                #print  eval(splitted[1])
				print "%s"%res
				#sss.move("base", eval(splitted[1]))
				sss.move("base", eval(res))
		elif splitted[0] == "move_tray":
			sss.move("tray",eval(splitted[1]))
		elif splitted[0] == "sleep":
			sss.sleep(eval(splitted[1]))
		elif splitted[0] == "move_arm":
			sss.move("arm",eval(splitted[1]),str2bool(splitted[2]))
		elif splitted[0] == "move_sdh":
			sss.move("sdh",eval(splitted[1]),str2bool(splitted[2]))
		elif splitted[0]== "move_head":
			sss.move("head",eval(splitted[1]))
		elif splitted[0]== "move_torso":
			sss.move("torso",eval(splitted[1]),str2bool(splitted[2]))

def handle_action_request(req):
	print "Received request: %s" % (req.action)
	path = roslib.packages.get_pkg_dir('AccompanyService')+"/src/actions/"+req.action
	#f = open("/home/patrizia/ros_workspace/AccompanyService/src/actions/"+req.action)
	f = open(path)
	lines = f.readlines()
	f.close()

	for line in lines:
		do(line,req.uid)
		
	#sss.say([req.action])                        #ok funza
	#sss.move("tray","down")
	#sss.move("tray","up")
	return 1

def action_server():
	rospy.init_node('ActionsService')
	s = rospy.Service('ActionsService',AccompanyAction,handle_action_request)
	print "Accompany Actions service running..."
	rospy.spin()

if __name__=="__main__":
	action_server()

