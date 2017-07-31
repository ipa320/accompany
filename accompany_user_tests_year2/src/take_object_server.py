#! /usr/bin/env python

import roslib; roslib.load_manifest('accompany_user_tests_year2')
import rospy
import actionlib

from accompany_user_tests_year2.msg import *
from simple_script_server import *
sss = simple_script_server()

import dynamic_reconfigure.client
from std_srvs.srv import Empty
import shelf

class TakeObjectServer:
	def __init__(self):
		self.server = actionlib.SimpleActionServer('take_object', TakeObjectShelfAction, self.execute, False)
		self.server.start()

	def execute(self, goal):
		goal.shelf_height = 1.44		
		shelf.shelf()
		self.server.set_succeeded()




if __name__ == '__main__':
	rospy.init_node('take_object_server')
	server = TakeObjectServer()
	rospy.spin()
