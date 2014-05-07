#!/usr/bin/python

import roslib
roslib.load_manifest('accompany_user_tests_year2')
import rospy
import math
import smach
import smach_ros
import numpy
import heapq
from simple_script_server import *  # import script
sss = simple_script_server()
#from sensor_msgs.msg import *
#from geometry_msgs.msg import *
from accompany_uva_msg.msg import *
import tf
from tf.transformations import *
from ScreenFormatting import *
import MySQLdb

db = MySQLdb.connect(host="localhost",
					user="accompanyUser",
					passwd="accompany",
					db="AccompanyTroyes")
cur = db.cursor()
cur.execute("""UPDATE Sensors SET value = True WHERE sensorId = 535""")	#need to be commented with scenario running
cur.execute("""SELECT value FROM Sensors WHERE sensorId=535""")
for followUser_condition in cur.fetchall():
	print "follow user condition:  ", followUser_condition[0]

class FollowUser(smach.State):
	def __init__(self):
		smach.State.__init__(self,
			outcomes=['succeeded','failed'])
#		rospy.Subscriber("/trackedHumans", TrackedHumans, self.callback)
		rospy.Subscriber("/leg_detection/detected_humans_laser", TrackedHumans, self.callback)
		self.listener = tf.TransformListener(True, rospy.Duration(20.0))
		self.user_position = PointStamped()
		self.user_speed = Vector3Stamped()
		self.tracking_user = False
		self.last_user_position = [0.0,0.0]
		
	def callback(self,msg):

#		print "msg.trackedHumans:",msg.trackedHumans
		for tracked_human in msg.trackedHumans:
			#if tracked_human.specialFlag == 1:
			if (self.tracking_user == False):
				self.last_user_position = [0.0,0.0]
				self.tracking_user = True
			speed = math.sqrt(tracked_human.speed.vector.x*tracked_human.speed.vector.x + tracked_human.speed.vector.y*tracked_human.speed.vector.y)
			if (speed > 0.2): ## todo check
				self.user_speed = tracked_human.speed
				self.user_position = tracked_human.location
#			print "user found"

		return

	def execute(self, userdata):
		sf = ScreenFormat("FollowUser")

		sss.set_light("yellow")
		while followUser_condition[0] == "1": ## todo: find some finishing criterion

			if self.user_position.point.x >= 1 and self.user_position.point.y >= -1 and self.user_position.point.y <= 1.2:#and self.user_speed.vector.x != 0 and self.user_speed.vector.y != 0:
			#if self.tracking_user == True:
				# check if the user moved enough and whether we have some significant movement speed
				dist = math.sqrt((self.last_user_position[0]-self.user_position.point.x)*(self.last_user_position[0]-self.user_position.point.x) +
						(self.last_user_position[1]-self.user_position.point.y)*(self.last_user_position[1]-self.user_position.point.y))
				self.last_user_position[0] = self.user_position.point.x
				self.last_user_position[1] = self.user_position.point.y
				speed = math.sqrt(self.user_speed.vector.x*self.user_speed.vector.x + self.user_speed.vector.y*self.user_speed.vector.y)
				if dist > 0.05: # and speed > 0.2:
					# compute a robot offset from user
					v_u = [self.user_speed.vector.x/speed, self.user_speed.vector.y/speed]	# normalized speed vector 2D
					n_u = [-self.user_speed.vector.y/speed, self.user_speed.vector.x/speed]	# normalized normal to speed vector 2D
					rx = self.user_position.point.x - 1*n_u[0] + v_u[0]*0.3
					ry = self.user_position.point.y - 1*n_u[1] + v_u[1]*0.3
					theta = math.atan2(self.user_speed.vector.y, self.user_speed.vector.x)
#					rx = self.user_position.point.x + 0.8*math.cos(theta)
#					ry = self.user_position.point.y + 0.8*math.sin(theta)
					print "human position: ", self.user_position.point.x, self.user_position.point.y
					# let the robot move there
					print "robot gets the position:", [rx, ry, theta]
					handle_base=sss.move("base",[rx, ry, theta], blocking=False,mode='linear')
#					handle_base=sss.move("base",[rx, ry, theta], blocking=False)
					if rx < 0.5 and ry > -1:
						print "experiment is over."
						break
					else:
						print "robot moves to", [rx, ry, theta]
			self.user_position.point.x = 0
			self.user_position.point.y = 0
			rospy.sleep(0.1)

		sss.set_light("green")
		

                return 'succeeded'

class SM(smach.StateMachine):
	def __init__(self):
		smach.StateMachine.__init__(self,outcomes=['ended'])
		with self:
			smach.StateMachine.add('FOLLOW_USER', FollowUser(),
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
