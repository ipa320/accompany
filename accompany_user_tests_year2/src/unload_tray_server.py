#! /usr/bin/env python

import roslib; roslib.load_manifest('accompany_user_tests_year2')
import rospy
import actionlib

from accompany_user_tests_year2.msg import *
from simple_script_server import *
sss = simple_script_server()

class UnloadTrayServer:
	def __init__(self):
		self.server = actionlib.SimpleActionServer('unload_tray', UnloadTrayAction, self.execute, False)
		self.server.start()

	def execute(self, goal):
		sss.set_light("blue")
		#TODO fill Thiagos code here
		success = True
		
		if success:
			self.server.set_succeeded()
		else:
			self.server.set_aborted()


if __name__ == '__main__':
	rospy.init_node('unload_tray_server')
	server = UnloadTrayServer()
	rospy.spin()
