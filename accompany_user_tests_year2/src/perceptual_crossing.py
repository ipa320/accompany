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
		self.user_position = PointStamped()
		self.user_speed = Vector3Stamped()
		self.tracking_user = False
		self.last_user_position = [0.0,0.0]
		
	def callback(self,msg):
		# go through list of detections and append them to detection list
		print "callback"
		print msg.trackedHumans
		for tracked_human in msg.trackedHumans:
			if tracked_human.identity == "user":  # some conditions that identifies the user
				if (self.tracking_user == False):
					self.last_user_position = [0.0,0.0]
				self.tracking_user = True
				self.user_position = tracked_human.location
				speed = math.sqrt(tracked_human.speed.vector.x*tracked_human.speed.vector.x + tracked_human.speed.vector.y*tracked_human.speed.vector.y)
				if (speed > 0.1): ## todo check
					self.user_speed = tracked_human.speed
				print "user found"
		return

	def execute(self, userdata):
		sf = ScreenFormat("FollowUser")
		
		sss.set_light("yellow")
		while True: ## todo: find some finishing criterion
			if self.tracking_user == True:
				# check if the user moved enough and whether we have some significant movement speed
				dist = math.sqrt((self.last_user_position[0]-self.user_position.location.point.x)*(self.last_user_position[0]-self.user_position.location.point.x) +
						(self.last_user_position[1]-self.user_position.location.point.y)*(self.last_user_position[1]-self.user_position.location.point.y))
				self.last_user_position[0] = self.user_position.location.point.x
				self.last_user_position[1] = self.user_position.location.point.y
				speed = math.sqrt(self.user_speed.vector.x*self.user_speed.vector.x + self.user_speed.vector.y*self.user_speed.vector.y)
				if dist > 0.2 And speed > 0.1:
					# compute a robot offset from user
					v_u = [self.user_speed.vector.x/speed, self.user_speed.vector.y/speed]	# normalized speed vector 2D
					n_u = [-self.user_speed.vector.y/speed, self.user_speed.vector.x/speed]	# normalized normal to speed vector 2D
					rx = self.user_position.location.point.x + n_u[0] + v_u[0]*0.3
					ry = self.user_position.location.point.y + n_u[1] + v_u[1]*0.3
					theta = math.atan2(self.user_speed.vector.y, self.user_speed.vector.x)
					# let the robot move there
					handle_base=sss.move("base",[rx, ry, theta], blocking=False)
			rospy.sleep(0.1)
		sss.set_light("green")
		
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
