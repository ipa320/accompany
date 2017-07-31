#!/usr/bin/python

import sys, os
import roslib; roslib.load_manifest('accompany_user_tests_year2')
import rospy
import actionlib
import accompany_user_tests_year2.msg

if __name__=='__main__':
	rospy.init_node('walkTogetherToDoorClient')
	#try:
	client = actionlib.SimpleActionClient('/walkTogetherToDoorServer', accompany_user_tests_year2.msg.EmptyAction)
	rospy.loginfo("Waiting for the walkTogetherToDoorServer server to start...")
	client.wait_for_server()
	rospy.loginfo("walkTogetherToDoorServer action server started, sending goal...")
	goal = accompany_user_tests_year2.msg.EmptyGoal()
	client.send_goal(goal)
	finished_before_timeout = client.wait_for_result()
	if finished_before_timeout:
		state = client.get_state()
		if state is 3:
			state = 'SUCCEEDED'
			rospy.loginfo("accompany_user_tests_year2 action finished: %s " % state)
		else:
			rospy.loginfo("accompany_user_tests_year2 action finished: %s " % state)
	else:
		rospy.loginfo("accompany_user_tests_year2 action did not finish before the time out.")
		#client.get_result()
	#except:
	#	print('EXCEPTION THROWN')
	#	print('Aborting cleanly')
	#	os._exit(1)
