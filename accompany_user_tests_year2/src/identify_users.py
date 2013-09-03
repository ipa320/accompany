#!/usr/bin/python


#TODO transformation from coordinate fmrame of camera to map necessary

############### PARAMETER SETTINGS #######################
import roslib
roslib.load_manifest('cob_generic_states_experimental')
roslib.load_manifest('accompany_user_tests_year2')
import rospy
import math
import smach
import smach_ros
import random
from cob_people_detection_msgs.msg import *
from accompany_uva_msg.msg import *
from ApproachPose import *
import sys
from tf import TransformListener
from tf.transformations import euler_from_quaternion
import tf
import time
import threading
from GoToGoalGeneric import *
from geometry_msgs.msg import Pose2D

from GoToUtils import *
from ScreenFormatting import *
############### PARAMETER SETTINGS #######################


# value defines perimeter around blocked goals, that are supposed to be
# approached
global GOAL_PERIMETER
GOAL_PERIMETER=1.3 #[m]
# threshold defines whether a goal is approached close enough
global APPROACHED_THRESHOLD
APPROACHED_THRESHOLD=0.4 #[m]
# threshold defines whether a goal is similar to another goal 
global SIMILAR_GOAL_THRESHOLD
SIMILAR_GOAL_THRESHOLD=0.3 #[m]
# predefined goals that can be used throughout the script ( couch is where the person is supposed to be in the beginning, kitchen is the position where something is grabbed afterwards)
global PREDEFINED_GOALS
couch_pose=Pose2D()
couch_pose.x=0.2 #1.2
couch_pose.y=1.4 #-0.3
couch_pose.theta=math.pi/2.0 #0.0
kitchen_pose=Pose2D()
kitchen_pose.x=4.8 #0
kitchen_pose.y=1.0 #0
kitchen_pose.theta=0.0 #math.pi

PREDEFINED_GOALS={
          "couch":  couch_pose,
        "kitchen": kitchen_pose
        }

# topic where face labels and positions are obtained by face recognition
global TOPIC_PEOPLE_DETECTION
TOPIC_PEOPLE_DETECTION="/cob_people_detection/detection_tracker/face_position_array"

# topic where person positions are provided, tracked by tha accompany tracker
global TOPIC_TRACKED_HUMANS
TOPIC_TRACKED_HUMANS="/trackedHumans"

#  bounds of the map where randomized positions are approached if necessary
global MAP_BOUNDS
MAP_BOUNDS=[-1.5,4.5,-0.5,2.5] # x_min,x_max,y_min,y_max
#MAP_BOUNDS=[-0.5,4.0,-4.5,0.5] # x_min,x_max,y_min,y_max

#  set to true if you want to double check if the person is still at the goal
global DOUBLECHECK
DOUBLECHECK=False



class CallName(smach.State):
	def __init__(self):
		smach.State.__init__(self,
		outcomes=['finished','failed'],
			input_keys=[],
			output_keys=[])
		rospy.Subscriber(TOPIC_PEOPLE_DETECTION, DetectionArray, self.callback)
		self.detections=      list()
		self.false_detections=list()
		self.utils=Utils()

	def callback(self,msg):
		# go through list of detections and append them to detection list
		if len(msg.detections)>0:
			#clear detection list
			del self.detections[:]
		for i in xrange(len(msg.detections)):
			self.detections.append(msg.detections[i])
		return

	def execute(self, userdata):
		sf = ScreenFormat("CallName")
		rospy.sleep(3)
		detections = self.detections
		attempts = 0
		while len(detections)==0 and attempts<90:
			rospy.sleep(0.1)
			attempts = attempts + 1
			detections = self.detections
			
		if len(detections)==0:
			rospy.loginfo("did not detect anybody")
			return 'finished'

		for det in detections:
			if det.pose.pose.position.z < 2.0:
				sss.say(['Hello %s.'%str(det.label)])
		return 'finished'


class SelectNextUserLocation(smach.State):
	def __init__(self):
		smach.State.__init__(self,
				outcomes=['new_goal','finished','failed'],
				input_keys=['callback_config','predefinitions'],
				output_keys=['current_goal', 'use_perimeter_goal'])
		self.generic_listener=GenericListener(target_frame="/map")
		self.utils=Utils()

	def execute(self, userdata):
		sf = ScreenFormat("SelectNextUserLocation")
		try:
			tl = get_transform_listener()
			tl.waitForTransform('/map', '/room_frame', rospy.Time(0), rospy.Duration(10))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			rospy.logwarn("SelectNextUserLocation: transform map - room_frame not available")
		self.generic_listener.reset()
		self.generic_listener.set_config(userdata.callback_config)

		# wait for callback
		rospy.sleep(3)
		detections = self.generic_listener.get_detections()
		attempts = 0
		while len(detections)==0 and attempts<1500:
			rospy.sleep(0.01)
			attempts = attempts + 1
			detections = self.generic_listener.get_detections()

		if len(detections)==0:
			rospy.logerr("No detections received, terminating.")
			sss.say(["I am so sad. I did not find any user. What a pity."])
			return 'failed'
				
		rospy.loginfo("received %i detections"%len(detections))
		print "detections:"
		print detections

		# determine robot pose		
		for i in xrange(10):
			(trafo_possible,robot_pose,quaternion)=self.utils.getRobotPose(get_transform_listener())
			rospy.sleep(0.2)
			if trafo_possible==True:
				break
		
		# select closest next person
		closest_pose = False
		minimum_distance_squared = 100000.0
		for name,pose in detections:
			if name == str(""):
				if trafo_possible==True:
					dist_squared = (pose.x-robot_pose[0])*(pose.x-robot_pose[0])+(pose.y-robot_pose[1])*(pose.y-robot_pose[1])
					if dist_squared < minimum_distance_squared:
			  			minimum_distance_squared = dist_squared
			  			closest_pose = pose
		  		else:
		  			closest_pose = pose
		  			break

		if closest_pose != False:
			current_goal = Pose2D()
			current_goal.x = closest_pose.x
			current_goal.y = closest_pose.y
			current_goal.theta = 0.0
			userdata.current_goal = current_goal
			userdata.use_perimeter_goal=True
			return 'new_goal'
		
		sss.say(["Great, I have identified all unknown users."])
		return 'finished'


class GoToGoal(smach.State):
	def __init__(self):
		smach.State.__init__(self,
				outcomes=['finished','failed'],
				input_keys= ['predefinitions','current_goal','use_perimeter_goal'],
				output_keys=['current_goal','use_perimeter_goal'])
		self.utils=Utils()

	def get_perimeter_goal(self,goal,radius):
		rospy.loginfo("Computing goal on perimeter")

		rotational_sampling_step = 10.0/180.0*math.pi
		rospy.wait_for_service('map_accessibility_analysis/map_perimeter_accessibility_check',10)
		try:
			get_approach_pose = rospy.ServiceProxy('map_accessibility_analysis/map_perimeter_accessibility_check', CheckPerimeterAccessibility)
			res = get_approach_pose(goal, radius, rotational_sampling_step)
			valid_poses=res.accessible_poses_on_perimeter
		except rospy.ServiceException, e:
			rospy.logwarn("Service call failed: %s",e)
			print "logwarn  returing false"
			return False
        
		if len(valid_poses) == 0:
			return -1

		# try for a while to get robot pose # TODO check if this is necessary
		for i in xrange(10):
			(trafo_possible,robot_pose,quaternion)=self.utils.getRobotPose(get_transform_listener())
			rospy.sleep(0.2)
			if trafo_possible==True:
				current_pose=Pose2D()
				current_pose.x=robot_pose[0]
				current_pose.y=robot_pose[1]
				break

		if trafo_possible==True:
			closest_pose = Pose2D()
			minimum_distance_squared = 100000.0
			for pose in valid_poses:
				dist_squared = (pose.x-current_pose.x)*(pose.x-current_pose.x)+(pose.y-current_pose.y)*(pose.y-current_pose.y)
				if dist_squared < minimum_distance_squared:
					minimum_distance_squared = dist_squared
					closest_pose = pose
			return closest_pose
		else:
			print "logwarn  returing false"
			rospy.logwarn("Could not get current robot pose - taking first pose in list")
			return valid_poses[0]
			#handle_base = sss.move("base", pose,blocking=block_program)
			#print "commanding move to current goal"

	def execute(self, userdata):
		sf = ScreenFormat("GoToGoal")
		rospy.loginfo("navigating to current goal")

		if userdata.use_perimeter_goal==True:
			radius_factor = 1.0
			while radius_factor < 1.75:
				perimeter_goal=self.get_perimeter_goal(userdata.current_goal,radius_factor*userdata.predefinitions["goal_perimeter"])
				#print "perimeter_goal=", perimeter_goal, "	radius_factor=", radius_factor
				if perimeter_goal!=False:
					if perimeter_goal==-1:
						radius_factor = radius_factor * 1.2
					else: 
						userdata.current_goal=perimeter_goal
						print "perimeter_goal=", perimeter_goal, "   radius=", radius_factor*userdata.predefinitions["goal_perimeter"]
						break
				else:
					rospy.loginfo("Commanding move to goal directly, as accessibility check is not available")
					break
		pose=list()
		pose.append(float(userdata.current_goal.x))
		pose.append(float(userdata.current_goal.y))
		pose.append(float(userdata.current_goal.theta))
		handle_base = sss.move("base", pose, blocking=True)
		rospy.loginfo("Commanding move to current goal")
		return 'finished'


class Rotate(smach.State):
	def __init__(self):
		smach.State.__init__(self,
				outcomes=['finished','failed'],
				input_keys=[],
				output_keys=[])

	def execute(self, userdata):
		sf = ScreenFormat("Rotate")
		
		rel_pose=list()
		rel_pose.append(0)
		rel_pose.append(0)
		rel_pose.append(0.1)
		for i in xrange(70):
			handle_base = sss.move_base_rel("base", rel_pose, blocking=True)
		
		return 'finished'


class IdentifyUsersRandomSearch(smach.StateMachine):
	def __init__(self):
		smach.StateMachine.__init__(self, 
								outcomes=['failed','finished'],
								input_keys=['predefinitions','callback_config','use_perimeter_goal'],
								output_keys=[])

		with self:
			smach.StateMachine.add("RANDOMGOAL", SetRandomGoal(),
								transitions={'failed':'failed',
											'finished':'GOTOGOAL'})
			
			smach.StateMachine.add("GOTOGOAL", GoToGoal(),
								transitions={'failed':'failed',
											'finished':'ROTATE'})
			
			smach.StateMachine.add("ROTATE", Rotate(),
								transitions={'failed':'failed',
											'finished':'RANDOMGOAL'})
			
			
class IdentifyUsersGuidedSearch(smach.StateMachine):
	def __init__(self):
		smach.StateMachine.__init__(self, 
								outcomes=['failed','finished'],
								input_keys=['predefinitions','callback_config','use_perimeter_goal'],
								output_keys=[])
		with self:
			smach.StateMachine.add("SELECTNEXTUSER", SelectNextUserLocation(),
								transitions={'failed':'failed',
                                            'finished':'finished',
											'new_goal':'GOTONEXTUSER'})
			
			smach.StateMachine.add("GOTONEXTUSER", GoToGoal(),
								transitions={'failed':'failed',
											'finished':'CALLNAME'})#'SELECTNEXTUSER'})
			
			smach.StateMachine.add("CALLNAME", CallName(),
								transitions={'failed':'failed',
											'finished':'SELECTNEXTUSER'})



def start_prompt():
	print "---------------------------------------------------------"
	print "plese start script with one of the follorwing arguments :"
	print "people_detection - use Face Recognition only for user approach"
	print "tracked_humans - use Face Recognition for recognition but tracked_humans for user approach"
	print "---------------------------------------------------------"
	sys.exit(1)


if __name__=='__main__':
	try:
		if len(sys.argv)<2:
			start_prompt()
		else:
			flag=sys.argv[1]
		if len(flag)==0:
			start_promt()
	
		rospy.init_node('IdentifyUsers')
		time.sleep(1)
		
		if flag=="people_detection":
			rospy.loginfo("Running identify users with people detections")
			sm = IdentifyUsersRandomSearch()
			sm.userdata.use_perimeter_goal=False
		elif flag=="tracked_humans":
			rospy.loginfo("Running identify users with tracked humans")
			sm = IdentifyUsersGuidedSearch()
			sm.userdata.callback_config={"argname_frame":["location","header","frame_id"],
										"msg_element":"trackedHumans",
										"argname_label":["identity"],
										"argname_position":["location","point"],
										"argname_header":["location","header"],
										"topicname":TOPIC_TRACKED_HUMANS,
										"msgclass":TrackedHumans}
			sm.userdata.use_perimeter_goal=True
		else:
			start_prompt()
		
		sm.userdata.predefinitions={"predefined_goals":PREDEFINED_GOALS,
									"map_bounds":MAP_BOUNDS,
									"double_check":DOUBLECHECK,
									"approached_threshold":APPROACHED_THRESHOLD,
									"similar_goal_threshold":SIMILAR_GOAL_THRESHOLD,
									"goal_perimeter":GOAL_PERIMETER}
	
		
		sis = smach_ros.IntrospectionServer('SM', sm, 'SM')
		sis.start()
		outcome = sm.execute()
		rospy.spin()
		sis.stop()
	except:
		print"exception"
		os._exit(1)
