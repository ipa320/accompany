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
from geometry_msgs.msg import PolygonStamped
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
#		rospy.Subscriber("/scan_top", LaserScan, self.callback)
		rospy.Subscriber("/leg_detection/detected_humans_laser", PolygonStamped, self.callback)
		self.listener = tf.TransformListener(True, rospy.Duration(20.0))
		self.human_x_relative=0
		self.human_y_relative=0
		
	def callback(self,msg):

#		print "msg.polygon.points:",msg.polygon.points
		self.human_x_relative=msg.polygon.points[0].x
		self.human_y_relative=msg.polygon.points[0].y
#		print "x,y:" , self.human_x_relative, self.human_y_relative
		return

	def execute(self, userdata):
		sf = ScreenFormat("FollowUser")

		sss.set_light("yellow")

		while 1:
			if self.human_x_relative != 0 and self.human_y_relative != 0:
				try:
					t = rospy.Time(0)
					self.listener.waitForTransform('/map', '/base_link', t,rospy.Duration(10))
					#robot_pose = self.listener.lookupTransform('/map', '/base_link',t)
					(robot_pose_translation, robot_pose_rotation) = self.listener.lookupTransform('/map', '/base_link',t)
					#print "robot pose: ", robot_pose[0][0], robot_pose[0][1]
				except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
					print "Could not lookup robot pose: %s" %e
				robot_pose_rotation_euler = tf.transformations.euler_from_quaternion(robot_pose_rotation, 'rzyx')
				print "robot pose: ", robot_pose_translation[0],robot_pose_translation[1],robot_pose_rotation_euler[0]
				robot_theta=robot_pose_rotation_euler[0]*180/math.pi 
				dist = math.hypot(self.human_x_relative,self.human_y_relative)
				angle_relative = math.atan2(self.human_y_relative,self.human_x_relative)
				speed_human=math.hypot(self.human_x_relative,self.human_y_relative)
				print "human position(to robot): [", self.human_x_relative, ", ", self.human_y_relative, "]"
				angle=robot_theta - angle_relative
				delta_x=math.cos(dist)
				delta_y=math.sin(dist)
				print "delta", delta_x, delta_y
				human_x=robot_pose_translation[0] + delta_x
				human_y=robot_pose_translation[1] - delta_y
				print "human position:[" ,human_x,",",human_y,"]" 
#			print "distance=",dist
#			print "speed=:",speed_human
#			if dist > 0 and speed_human > 0:
#
#				v_u = [human_x/speed_human, human_y/speed_human]
#				n_u = [-human_y/speed_human, human_x/speed_human]
#				rx= human_x+n_u[0]*1+v_u[0]*0.5
#				ry= human_y+n_u[1]*1+v_u[1]*0.5
#				theta = math.atan2(human_y, human_x)
#
#				handle_base=sss.move("base",[rx, ry, theta], blocking=False,mode='linear')
#				if rx > 8 and ry < 0.8:
#					print "robot stops moving"
#					break
#				else:
#					print "robot moves to", [rx, ry, theta]
				self.human_x_relative=0
				self.human_y_relative=0
			else:
				print "no human detected"
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
