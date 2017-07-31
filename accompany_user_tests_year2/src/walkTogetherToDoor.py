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
#import MySQLdb

#db = MySQLdb.connect(host="localhost",
#					user="accompanyUser",
#					passwd="accompany",
#					db="AccompanyTroyes")
#cur = db.cursor()
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

	def execute(self, userdata):
		sf = ScreenFormat("GoToUser")

		#sss.move("head", "front", False)
		#first part
		#rospy.sleep(10)
		#handle_base=sss.move("base",[0.588, -0.979, 2.2],True,mode='linear')
		#handle_base.wait()
		#handle_base=sss.move("base",[0.733, -1.247, 2.2],True,mode='linear')
		#handle_base.wait()
		#handle_base=sss.move("base",[0.588, -0.979, 2.2],True,mode='linear')
		#handle_base.wait()

		#handle_base=sss.move("base",[0.733, -1.247, 2.2],True,mode='linear')
		#handle_base.wait()
		#handle_base=sss.move("base",[0.588, -0.979, 2.2],True,mode='linear')
		#handle_base=sss.move("base",[0.22, -1.220, 0.918],False,mode='linear')
		#sss.set_light("yellow")
		#sss.move("torso", [[-0.1,-0.2,-0.15]], True)
		#handle_base.wait()
		#sss.set_light("breathing green")
		#rospy.sleep(3)
		#sss.move("torso", "home", True)	
		#sss.set_light("flashing yellow")
		
		return 'succeeded'

class FollowUser(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['succeeded','failed'])
		#rospy.Subscriber("/trackedHumans", TrackedHumans, self.callback)
		rospy.Subscriber("/leg_detection/detected_humans_laser", TrackedHumans, self.callback)
		self.user_position = PointStamped()
		self.user_speed = Vector3Stamped()
		self.tracking_user = False
		self.last_user_position = [0.0,0.0]

		# tracking area (only humans in this area are considered)
		self.min_x = 1.7
		self.max_x = 5.5
		self.min_y = -2.5
		self.max_y = -0.5
		
	def callback(self,msg):

		# read out current robot pose
		(robot_pose_translation, robot_pose_rotation, robot_pose_rotation_euler) = currentRobotPose()
		if robot_pose_translation==None:
			return
		min_distance_to_robot = 1000000.0
		tracking_user = False

#		print "msg.trackedHumans:",msg.trackedHumans
		for tracked_human in msg.trackedHumans:
			#if tracked_human.specialFlag == 1:
			speed = math.sqrt(tracked_human.speed.vector.x*tracked_human.speed.vector.x + tracked_human.speed.vector.y*tracked_human.speed.vector.y)
			distance_to_robot = math.sqrt((robot_pose_translation[0]-tracked_human.location.point.x)*(robot_pose_translation[0]-tracked_human.location.point.x) + (robot_pose_translation[1]-tracked_human.location.point.y)*(robot_pose_translation[1]-tracked_human.location.point.y))
			if (distance_to_robot>0.6 and distance_to_robot<min_distance_to_robot and speed > 0.1 and tracked_human.location.point.x >= self.min_x and tracked_human.location.point.x <= self.max_x and tracked_human.location.point.y <= self.max_y and tracked_human.location.point.y >= self.min_y): ## todo check
				min_distance_to_robot = distance_to_robot
				self.user_speed = tracked_human.speed
				self.user_position = tracked_human.location
				if (self.tracking_user == False):
					self.last_user_position = [0.0,0.0]
					tracking_user = True
					self.last_user_speed = [0.0,0.0]
#			print "user found"
		if tracking_user == True:
			self.tracking_user = True
			print "self.user_position", self.user_position

		return

	def execute(self, userdata):
		sf = ScreenFormat("FollowUser")

		print "before the while loop"
		#while followUser_condition[0] == "1": ## todo: find some finishing criterion
		while True:
			if self.user_position.point.x >= self.min_x and self.user_position.point.x <= self.max_x and self.user_position.point.y <= self.max_y and self.user_position.point.y >= self.min_y:  #and self.user_speed.vector.x != 0 and self.user_speed.vector.y != 0:
			#if self.tracking_user == True:
				# check if the user moved enough and whether we have some significant movement speed
				dist = math.sqrt((self.last_user_position[0]-self.user_position.point.x)*(self.last_user_position[0]-self.user_position.point.x) +
						(self.last_user_position[1]-self.user_position.point.y)*(self.last_user_position[1]-self.user_position.point.y))
				self.last_user_position[0] = self.user_position.point.x
				self.last_user_position[1] = self.user_position.point.y
				speed = math.sqrt(self.user_speed.vector.x*self.user_speed.vector.x + self.user_speed.vector.y*self.user_speed.vector.y)
				if dist > 0.05: # and speed > 0.2:
					print "let's move"
					# compute a robot offset from user (2 variants, left and right, take the one which is closer)
					v_u = [self.user_speed.vector.x/speed, self.user_speed.vector.y/speed]	# normalized speed vector 2D
					n_u = [-self.user_speed.vector.y/speed, self.user_speed.vector.x/speed]	# normalized normal to speed vector 2D
					rx_1 = self.user_position.point.x - 0.8*n_u[0] + v_u[0]*0.4
					ry_1 = self.user_position.point.y - 0.8*n_u[1] + v_u[1]*0.4
					rx_2 = self.user_position.point.x + 0.8*n_u[0] + v_u[0]*0.4
					ry_2 = self.user_position.point.y + 0.8*n_u[1] + v_u[1]*0.4
					theta = math.atan2(self.user_speed.vector.y, self.user_speed.vector.x)
					last_theta = math.atan2(self.last_user_speed[1], self.last_user_speed[0])
#					if math.fabs(last_theta - theta) < 30 * math.pi / 180:
#						theta = last_theta
#					print "diff:" , math.fabs(last_theta - theta)
					
					rx = rx_1
					ry = ry_1

					# read out current robot pose
					(robot_pose_translation, robot_pose_rotation, robot_pose_rotation_euler) = currentRobotPose()
					if (robot_pose_translation != None):
						dist_1 = (robot_pose_translation[0]-rx_1)*(robot_pose_translation[0]-rx_1) + (robot_pose_translation[1]-ry_1)*(robot_pose_translation[1]-ry_1)
						dist_2 = (robot_pose_translation[0]-rx_2)*(robot_pose_translation[0]-rx_2) + (robot_pose_translation[1]-ry_2)*(robot_pose_translation[1]-ry_2)
						if dist_2 < dist_1:
							rx = rx_2
							ry = ry_2
					else:
						rospy.logwarn("Could not read out robot pose.")

					print "human position: ", self.user_position.point.x, self.user_position.point.y
					# let the robot move there
					print "robot gets the position:", [rx, ry, theta]
					#sss.set_light("flashing yellow")
					handle_base=sss.move("base",[rx, ry, theta], blocking=False, mode='linear')
#					handle_base=sss.move("base",[rx, ry, theta], blocking=False)
					if self.user_position.point.x > 2 and self.user_position.point.y < -1.7:
						print "finished walk with me state"
						break
					#else:
					#	print "robot moves to", [rx, ry, theta]
				self.last_user_speed[0] = self.user_speed.vector.x
				self.last_user_speed[1] = self.user_speed.vector.y		
			#self.user_position.point.x = 0
			#self.user_position.point.y = 0
			rospy.sleep(0.05)
		return 'succeeded'


class LetUserEnterDoor(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['succeeded','failed'])

	def execute(self, userdata):
		sf = ScreenFormat("LetUserEnterDoor")

		# let user enter door first
		#sss.set_light("flashing yellow")
		handle_base=sss.move("base",[3.085, -2.660, -0.321],False)
		#sss.move("torso", [[0.0,-0.2,0.0]], False)
		handle_base.wait()
		#rospy.sleep(2)
		#sss.move("torso", "home", True)

		# move through door
		#handle_base=sss.move("base",[4.437, -2.736, -1.319],True,mode='linear')
		#handle_base.wait()
		#handle_base=sss.move("base",[4.746, -4.201,-1.303],True,mode='linear')
		#handle_base.wait()
		#sss.set_light("white")
		return 'succeeded'


class SM_walkTogetherToDoor(smach.StateMachine):
	def __init__(self):
		smach.StateMachine.__init__(self,outcomes=['ended'])
		with self:
			smach.StateMachine.add('GO_TO_USER', GoToUser(),
			transitions={'succeeded':'FOLLOW_USER',
					'failed':'ended'})

			smach.StateMachine.add('FOLLOW_USER', FollowUser(),
			transitions={'succeeded':'LET_USER_ENTER_DOOR',
					'failed':'ended'})

			smach.StateMachine.add('LET_USER_ENTER_DOOR', LetUserEnterDoor(),
			transitions={'succeeded':'ended',
					'failed':'ended'})

if __name__=='__main__':
	try:
		rospy.init_node('walkTogetherToDoor')
			
		sm = SM_walkTogetherToDoor()
		sis = smach_ros.IntrospectionServer('SM_walkTogetherToDoor', sm, 'SM_walkTogetherToDoor')
		sis.start()
		outcome = sm.execute()
		#rospy.spin()
		sis.stop()
	except:
		print('EXCEPTION THROWN')
		print('Aborting cleanly')
		os._exit(1)
