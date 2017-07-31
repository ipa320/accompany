#!/usr/bin/python

import roslib
roslib.load_manifest('accompany_user_tests_year2')
import rospy
import math
import smach
import smach_ros
import numpy
import threading
import heapq
from simple_script_server import *  # import script
sss = simple_script_server()
#from sensor_msgs.msg import *
#from geometry_msgs.msg import *
from accompany_uva_msg.msg import *
from accompany_common import *
import tf
from tf.transformations import *
from ScreenFormatting import *
import MySQLdb

db = MySQLdb.connect(host="10.0.1.207",
					user="accompanyUser",
					passwd="accompany",
					db="AccompanyTroyes")
cur = db.cursor()
#cur.execute("""UPDATE Sensors SET value = True WHERE sensorId = 535""")	#need to be commented with scenario running
#cur.execute("""SELECT value FROM Sensors WHERE sensorId=535""")
#for followUser_condition in cur.fetchall():
#	print "follow user condition:  ", followUser_condition[0]


def currentRobotPose():
	# read out current robot pose
	try:
		listener = get_transform_listener()
		t = rospy.Time(0)
		listener.waitForTransform('/map', '/base_link', t, rospy.Duration(10))
		(robot_pose_translation, robot_pose_rotation) = listener.lookupTransform('/map', '/base_link', t)
	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
		print "Could not lookup robot pose: %s" %e
		return (None, None, None)
	robot_pose_rotation_euler = tf.transformations.euler_from_quaternion(robot_pose_rotation, 'rzyx') # yields yaw, pitch, roll
	return (robot_pose_translation, robot_pose_rotation, robot_pose_rotation_euler)


class GoToUser(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['succeeded','failed'])
		rospy.Subscriber("/trackedHumans", TrackedHumans, self.callback2)
		self.user_position = PointStamped()

		self.min_x2 = -0.3
		self.max_x2 = 2.0
		self.min_y2 = -3.2
		self.max_y2 = 0.0
		self.tracking_user2 = False
		self.movereminder = False

		
	def callback2(self,msg):
		self.min_x2 = -0.3
		for tracked_human in msg.trackedHumans:
			if tracked_human.location.point.x >= self.min_x2 and tracked_human.location.point.x <= self.max_x2 and tracked_human.location.point.y <= self.max_y2 and tracked_human.location.point.y >= self.min_y2 and self.movereminder == False:
				print "In the zone ... moving"
				self.tracking_user2 = True
		if self.tracking_user2 == False and self.movereminder == False :
			print "Not in the zone"
		
	def execute(self, userdata):


		if self.tracking_user2 == True and self.movereminder == False :
			self.movereminder == True
			handle_base=sss.move("base",[1.665, -2.763, 1.674],True)

			return 'succeeded'
		rospy.sleep(1)
		return 'failed'
		###hackraw_input("orientation")
		handle_base=sss.move("base",[0.846, -1.556, 1.683],True)
		####hackraw_input("enter sofa")
		return 'succeeded'
	
		

class FollowUser(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['succeeded','failed'])
##		rospy.Subscriber("/leg_detection/detected_humans_laser", TrackedHumans, self.callback)
		self.user_position = PointStamped()
		self.user_speed = Vector3Stamped()
		self.tracking_user = False
		self.last_user_position = [0.0,0.0]

		# tracking area (only humans in this area are considered)
		self.min_x = 0.0
		self.max_x = 2.0
		self.min_y = -3.0
		self.max_y = 0.0
		
	def callback(self,msg):
### perceptual crossing beginning
		# read out current robot pose
#		(robot_pose_translation, robot_pose_rotation, robot_pose_rotation_euler) = currentRobotPose()
#		if robot_pose_translation==None:
#			return
#		min_distance_to_robot = 1000000.0
#		tracking_user = False

#		for tracked_human in msg.trackedHumans:
#			speed = math.sqrt(tracked_human.speed.vector.x*tracked_human.speed.vector.x + tracked_human.speed.vector.y*tracked_human.speed.vector.y)
#			distance_to_robot = math.sqrt((robot_pose_translation[0]-tracked_human.location.point.x)*(robot_pose_translation[0]-tracked_human.location.point.x) + (robot_pose_translation[1]-tracked_human.location.point.y)*(robot_pose_translation[1]-tracked_human.location.point.y))
#			if (distance_to_robot>0.25 and distance_to_robot<min_distance_to_robot and speed > 0.15 and tracked_human.location.point.x >= self.min_x and tracked_human.location.point.x <= self.max_x and tracked_human.location.point.y <= self.max_y and tracked_human.location.point.y >= self.min_y): ## todo check
#				min_distance_to_robot = distance_to_robot
#				self.user_speed = tracked_human.speed
#				self.user_position = tracked_human.location
#				if (self.tracking_user == False):
#					self.last_user_position = [0.0,0.0]
#					tracking_user = True
#					self.last_user_speed = [0.0,0.0]
#		if tracking_user == True:
#			self.tracking_user = True
#			print "self.user_position", self.user_position

		return

	def execute(self, userdata):
		sf = ScreenFormat("FollowUser")
###########perceptual crossing part #######################
#		print "before the while loop"
#		while True:
#			if self.user_position.point.x >= self.min_x and self.user_position.point.x <= self.max_x and self.user_position.point.y <= self.max_y and self.user_position.point.y >= self.min_y:  #and self.user_speed.vector.x != 0 and self.user_speed.vector.y != 0:
#			#if self.tracking_user == True:
#				# check if the user moved enough and whether we have some significant movement speed
#				dist = math.sqrt((self.last_user_position[0]-self.user_position.point.x)*(self.last_user_position[0]-self.user_position.point.x) +
#						(self.last_user_position[1]-self.user_position.point.y)*(self.last_user_position[1]-self.user_position.point.y))
#				self.last_user_position[0] = self.user_position.point.x
#				self.last_user_position[1] = self.user_position.point.y
#				speed = math.sqrt(self.user_speed.vector.x*self.user_speed.vector.x + self.user_speed.vector.y*self.user_speed.vector.y)
#				if dist > 0.05: # and speed > 0.2:
#					print "let's move"
#					# compute a robot offset from user (2 variants, left and right, take the one which is closer)
#					v_u = [self.user_speed.vector.x/speed, self.user_speed.vector.y/speed]	# normalized speed vector 2D
#					n_u = [-self.user_speed.vector.y/speed, self.user_speed.vector.x/speed]	# normalized normal to speed vector 2D
#					rx_1 = self.user_position.point.x - 0.8*n_u[0] + v_u[0]*0.3
#					ry_1 = self.user_position.point.y - 0.8*n_u[1] + v_u[1]*0.3
#					rx_2 = self.user_position.point.x + 0.8*n_u[0] + v_u[0]*0.3
#					ry_2 = self.user_position.point.y + 0.8*n_u[1] + v_u[1]*0.3
#					theta = math.atan2(self.user_speed.vector.y, self.user_speed.vector.x)
#					last_theta = math.atan2(self.last_user_speed[1], self.last_user_speed[0])

					
#					rx = rx_1
#					ry = ry_1

					# read out current robot pose
#					(robot_pose_translation, robot_pose_rotation, robot_pose_rotation_euler) = currentRobotPose()
#					if (robot_pose_translation != None):
#						dist_1 = (robot_pose_translation[0]-rx_1)*(robot_pose_translation[0]-rx_1) + (robot_pose_translation[1]-ry_1)*(robot_pose_translation[1]-ry_1)
#						dist_2 = (robot_pose_translation[0]-rx_2)*(robot_pose_translation[0]-rx_2) + (robot_pose_translation[1]-ry_2)*(robot_pose_translation[1]-ry_2)
#						if dist_2 < dist_1:
#							rx = rx_2
#							ry = ry_2
#					else:
#						rospy.logwarn("Could not read out robot pose.")
#
#					print "human position: ", self.user_position.point.x, self.user_position.point.y
#					print "robot gets the position:", [rx, ry, theta]
#					handle_base=sss.move("base",[rx, ry, theta], blocking=False, mode='linear')
#					print self.user_position.point.y
#					if self.user_position.point.y > -2.5 and self.user_position.point.x < 2.0:
#						print "finished walk with me state"
#						break
#				self.last_user_speed[0] = self.user_speed.vector.x
#				self.last_user_speed[1] = self.user_speed.vector.y		
			#rospy.sleep(0.05)
		
############ perceptual crossing part end
		return 'succeeded'


class LetUserEnterLivingRoom(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['succeeded','failed'])

	def execute(self, userdata):
		sf = ScreenFormat("LetUserEnterLivingRoom")

		# let user enter door first
		handle_base=sss.move("base",[0.182, -0.635, 0.605],True)
		cur.execute("""UPDATE Sensors SET value = 'True' WHERE sensorId = 534""")
		handle_base=sss.move("base",[0.365, -0.447, 0.605],False,mode='linear')
		handle_base.wait()
		#sss.set_light("breathing green")
		rospy.sleep(2)

		
		return 'succeeded'


class SM_walkTogetherToSofa(smach.StateMachine):
	def __init__(self):
		smach.StateMachine.__init__(self,outcomes=['ended'])
		with self:
			smach.StateMachine.add('GO_TO_USER', GoToUser(),
			transitions={'succeeded':'FOLLOW_USER',
					'failed':'GO_TO_USER'})

			smach.StateMachine.add('FOLLOW_USER', FollowUser(),
			transitions={'succeeded':'LET_USER_ENTER_LIVING_ROOM',
					'failed':'ended'})

			smach.StateMachine.add('LET_USER_ENTER_LIVING_ROOM', LetUserEnterLivingRoom(),
			transitions={'succeeded':'ended',
					'failed':'ended'})

if __name__=='__main__':
	try:
		rospy.init_node('walkTogetherToSofa')
		sm = SM_walkTogetherToSofa()
		sis = smach_ros.IntrospectionServer('SM_walkTogetherToSofa', sm, 'SM_walkTogetherToSofa')
		sis.start()
		outcome = sm.execute()
		#rospy.spin()
		sis.stop()
	except:
		print('EXCEPTION THROWN')
		print('Aborting cleanly')
		os._exit(1)
