#!/usr/bin/python

import roslib
roslib.load_manifest('accompany_user_tests_year2')
import rospy
import math
import smach
import smach_ros
import numpy
import threading
from simple_script_server import *  # import script
sss = simple_script_server()
#from sensor_msgs.msg import *
#from geometry_msgs.msg import *
from accompany_uva_msg.msg import *
from accompany_user_tests_year2.msg import *
import accompany_common
import tf
from tf.transformations import *
from ScreenFormatting import *

import walkTogetherToDoor
import walkTogetherToKitchen
import walkTogetherToSofa
import shelf

import actionlib



class walkTogetherToDoorServer:
	def __init__(self):
		self.server = actionlib.SimpleActionServer('walkTogetherToDoorServer', EmptyAction, self.execute, False)
		self.server.start()

	def execute(self, goal):
		sm = walkTogetherToDoor.SM_walkTogetherToDoor()
		sis = smach_ros.IntrospectionServer('SM_walkTogetherToDoor', sm, 'SM_walkTogetherToDoor')
		sis.start()
		outcome = sm.execute()
		sis.stop()
		
		self.server.set_succeeded()

class walkTogetherToKitchenServer:
	def __init__(self):
		self.server = actionlib.SimpleActionServer('walkTogetherToKitchenServer', EmptyAction, self.execute, False)
		self.server.start()

	def execute(self, goal):
		sm = walkTogetherToKitchen.SM_walkTogetherToKitchen()
		sis = smach_ros.IntrospectionServer('SM_walkTogetherToKitchen', sm, 'SM_walkTogetherToKitchen')
		sis.start()
		outcome = sm.execute()
		sis.stop()
		
		self.server.set_succeeded()


class walkTogetherToSofaServer:
	def __init__(self):
		self.server = actionlib.SimpleActionServer('walkTogetherToSofaServer', EmptyAction, self.execute, False)
		self.server.start()

	def execute(self, goal):
		sm = walkTogetherToSofa.SM_walkTogetherToSofa()
		sis = smach_ros.IntrospectionServer('SM_walkTogetherToSofa', sm, 'SM_walkTogetherToSofa')
		sis.start()
		outcome = sm.execute()
		sis.stop()
		
		self.server.set_succeeded()

class TakeObjectServer:
	def __init__(self):
		self.server = actionlib.SimpleActionServer('take_object', TakeObjectShelfAction, self.execute, False)
		self.server.start()

	def execute(self, goal):
		goal.shelf_height = 1.44		
		shelf.shelf()
		self.server.set_succeeded()




if __name__=='__main__':
	try:
		rospy.init_node('accompany_scenario_services')
		server1 = walkTogetherToDoorServer()
		server2 = walkTogetherToKitchenServer()
		server3 = walkTogetherToSofaServer()
		server = TakeObjectServer()
		rospy.spin()
	except:
		print('EXCEPTION THROWN')
		print('Aborting cleanly')
		os._exit(1)
