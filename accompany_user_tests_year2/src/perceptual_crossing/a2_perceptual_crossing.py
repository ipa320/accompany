#!/usr/bin/python

import roslib
roslib.load_manifest('accompany_user_tests_year2')
import rospy
import math
import smach
import smach_ros
from simple_script_server import *  # import script
sss = simple_script_server()

from accompany_uva_msg.msg import *

from ScreenFormatting import *

class Move_forward(smach.State):
	def __init__(self):
		smach.State.__init__(self,
			outcomes=['succeeded','failed'])

	def execute(self, userdata):
		sf = ScreenFormat("Move")
		
		sss.set_light("yellow")
		handle_base=sss.move("base",[5.3, 0.6, 3.089])
		handle_base.wait()
		handle_base=sss.move("base",[2, 0.6, 3.089])
		handle_base.wait()

		if handle_base.get_state() != 3:
		   print "base state = ",handle_base.get_state()
		   sss.set_light("red")
		   return 'failed' 
                sss.set_light("green")
                return 'succeeded'

class SM(smach.StateMachine):
	def __init__(self):
		smach.StateMachine.__init__(self,outcomes=['ended'])
		with self:
			smach.StateMachine.add('MOVE_FORWARD',Move_forward	(),
			transitions={'succeeded':'ended',
					'failed':'ended'})

if __name__=='__main__':
	try:
		rospy.init_node('Move')
		sm = SM()
		sis = smach_ros.IntrospectionServer('SM', sm, 'SM')
		sis.start()
		outcome = sm.execute()
		rospy.spin()
		sis.stop()
	except:
		print('EXCEPTION THROWN')
		print('Aborting cleanly')
		os._exit(1)
