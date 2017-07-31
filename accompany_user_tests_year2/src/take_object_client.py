#! /usr/bin/env python

import roslib; roslib.load_manifest('accompany_user_tests_year2')
import rospy
import actionlib

from accompany_user_tests_year2.msg import *

if __name__ == '__main__':
	rospy.init_node('take_object_client')
	client = actionlib.SimpleActionClient('take_object', TakeObjectShelfAction)
	client.wait_for_server()

	goal = TakeObjectShelfGoal()	
	# Fill in the goal here
	client.send_goal(goal)
	if not client.wait_for_result():
		print "not finished within time"
	print "result=", client.get_state()

