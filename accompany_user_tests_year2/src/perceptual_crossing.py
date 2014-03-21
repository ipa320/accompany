#!/usr/bin/python

import roslib
roslib.load_manifest('accompany_user_tests_year2')
import rospy
import smach
import smach_ros
from simple_script_server import *  # import script
sss = simple_script_server()

from accompany_uva_msg.msg import *

from ScreenFormatting import *

class Move(smach.State):
	def __init__(self):
		smach.State.__init__(self,
			outcomes=['succeeded','failed'])

	def execute(self, userdata):
		sf = ScreenFormat("Move")
		
		sss.set_light("yellow")
		handle_base=sss.move("base",[6.585, 2.7, -3.145])
		handle_base.wait()
		sss.say(["look at table position reached"])
		handle_base=sss.move("base",[5.55, 2, -2.758])
		handle_base.wait()
		sss.say(["unload sofa right position reached"])
		handle_base=sss.move("base",[5.285,3.975,-3.137])
		handle_base.wait()
		sss.say(["charging position reached"])
		
		if handle_base.get_state() != 3:
		   print "base state = ",handle_base.get_state()
		   sss.set_light("red")
		   return 'failed' 
                sss.set_light("green")
                return 'succeeded'

class FollowUser(smach.State):
	def __init__(self):
		smach.State.__init__(self,
			outcomes=['succeeded','failed'])
		rospy.Subscriber("/trackedHumans", TrackedHumans, self.callback)
		self.user_position = []
		
	def callback(self,msg):
		# go through list of detections and append them to detection list
		print "callback"
		print msg.trackedHumans
		if len(msg.trackedHumans) > 0:
			print "some person is in the message"
		return

	def execute(self, userdata):
		sf = ScreenFormat("FollowUser")
		
		sss.set_light("yellow")
		rospy.sleep(100)
		sss.set_light("green")
		
#		while True:
#			if self.user
		
#		handle_base=sss.move("base",[6.585, 2.7, -3.145], blocking=False)
#		rospy.sleep(2)
#		#sss.say(["look at table position reached"])
#		handle_base=sss.move("base",[5.55, 2, -2.758], blocking=False)
#		rospy.sleep(3)
#		#sss.say(["unload sofa right position reached"])
#		handle_base=sss.move("base",[6.585, 2.7, -3.145], blocking=False)
#		rospy.sleep(3)
#		#sss.say(["look at table position reached"])
				
#		if handle_base.get_state() != 3:
#		   print "base state = ",handle_base.get_state()
#		   sss.set_light("red")
#		   return 'failed' 
#               sss.set_light("green")
                return 'succeeded'


class SM(smach.StateMachine):
	def __init__(self):
		smach.StateMachine.__init__(self,outcomes=['ended'])
		with self:
			smach.StateMachine.add('FOLLOW_USER', FollowUser(),
			transitions={'succeeded':'ended',
					'failed':'ended'})

#			smach.StateMachine.add('MOVE_BACK_AND_FORTH',Move(),
#			transitions={'succeeded':'ended',
#					'failed':'ended'})

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
