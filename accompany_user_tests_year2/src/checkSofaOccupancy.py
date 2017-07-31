#!/usr/bin/python

import roslib
roslib.load_manifest('accompany_user_tests_year2')
import rospy
from simple_script_server import *  # import script
sss = simple_script_server()
#from sensor_msgs.msg import *
#from geometry_msgs.msg import *
from accompany_uva_msg.msg import *
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




class GoToChargingPosition:
	def __init__(self):
		rospy.Subscriber("/trackedHumans", TrackedHumans, self.callback)
		self.min_x = 0.0
		self.max_x = 3.0
		self.min_y = 0.0
		self.max_y = 2.0
			
		
	def callback(self,msg):
		tracking_user = False
		for tracked_human in msg.trackedHumans:
			if (tracked_human.location.point.x >= self.min_x and tracked_human.location.point.x <= self.max_x and tracked_human.location.point.y <= self.max_y and 	tracked_human.location.point.y >= self.min_y):
				# somebody is sitting at the sofa --> write to database
				cur.execute("""UPDATE Sensors SET value = 'True' WHERE sensorId = 588""")
				tracking_user = True
				print "Somebody is sitting at the sofa."
		if tracking_user == False:
			cur.execute("""UPDATE Sensors SET value = 'False' WHERE sensorId = 588""")
			print "The sofa is free."


if __name__=='__main__':
	try:
		rospy.init_node('SittingAtSofaSensor')
		
		gtcp = GoToChargingPosition()

		rospy.spin()
		
	except:
		print('EXCEPTION THROWN')
		print('Aborting cleanly')
		os._exit(1)
