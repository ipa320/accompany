#! /usr/bin/env python

import roslib; roslib.load_manifest('accompany_user_tests_year2')
import rospy
import actionlib

from accompany_user_tests_year2.msg import *

if __name__ == '__main__':
	rospy.init_node('unload_tray_client')
	client = actionlib.SimpleActionClient('unload_tray', UnloadTrayAction)
	client.wait_for_server()

	goal = UnloadTrayGoal()
	goal.table_height = 0.45

	# Fill in the goal here
	client.send_goal(goal)
	if not client.wait_for_result(rospy.Duration.from_sec(90.0)):
		print "not finished within time"
	print "result=", client.get_state()

