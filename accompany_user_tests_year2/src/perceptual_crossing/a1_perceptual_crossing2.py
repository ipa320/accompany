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


###############''WORKAROUND FOR TRANSFORMLISTENER ISSUE####################
_tl=None
_tl_creation_lock=threading.Lock()

def get_transform_listener():
    global _tl
    with _tl_creation_lock:
        if _tl==None:
            _tl=tf.TransformListener(True, rospy.Duration(40.0))
        return _tl
#################################################################################


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
		rospy.Subscriber("/trackedHumans", TrackedHumans, self.callback)
		self.user_position = PointStamped()
		self.min_x2 = 0.0
		self.max_x2 = 3.0
		self.min_y2 = 0.0
		self.max_y2 = 2.0
		self.tracking_user = False
			

		
	def callback(self,msg):
		for tracked_human in msg.trackedHumans:
			self.user_position = tracked_human.location
			self.tracking_user = True

	def execute(self, userdata):
		sf = ScreenFormat("GoToUser")
		while self.tracking_user == False:
			print "not tracked yet"
		print self.user_position.point.x
		print self.user_position.point.y
		while  not (self.user_position.point.x >= self.min_x2 and self.user_position.point.x <= self.max_x2 and self.user_position.point.y <= self.max_y2 and 	self.user_position.point.y >= self.min_y2):
			print "still not in the zone"
		cur.execute("""UPDATE Sensors SET value = 'True' WHERE sensorId = 588""")	
		return 'succeeded'


class SM(smach.StateMachine):
	def __init__(self):
		smach.StateMachine.__init__(self,outcomes=['ended'])
		with self:
			smach.StateMachine.add('GO_TO_USER', GoToUser(),
			transitions={'succeeded':'ended',
					'failed':'ended'})


if __name__=='__main__':
	try:
		rospy.init_node('Move')
		sm = SM()
		sis = smach_ros.IntrospectionServer('SM', sm, 'SM')
		sis.start()
		outcome = sm.execute()
		#rospy.spin()
		sis.stop()
	except:
		print('EXCEPTION THROWN')
		print('Aborting cleanly')
		os._exit(1)
