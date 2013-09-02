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
TOPIC_TRACKED_HUMANS="/accompany/TrackedHumans"

#  bounds of the map where randomized positions are approached if necessary
global MAP_BOUNDS
MAP_BOUNDS=[-1.5,4.5,-0.5,2.5] # x_min,x_max,y_min,y_max
#MAP_BOUNDS=[-0.5,4.0,-4.5,0.5] # x_min,x_max,y_min,y_max

#  set to true if you want to double check if the person is still at the goal
global DOUBLECHECK
DOUBLECHECK=False



class SelectRandomGoal(smach.State):
  def __init__(self):
    smach.State.__init__(self,
      outcomes=['selected','not_selected','failed'],
      input_keys=['current_goal'],
      output_keys=['current_goal'])

  def execute(self, userdata):
      sf = ScreenFormat("SelectRandomGoal")
      map_bounds=MAP_BOUNDS
      rand_x=random.uniform(map_bounds[0],map_bounds[1]) # x
      rand_y=random.uniform(map_bounds[2],map_bounds[3]) # y
      rand_theta=random.uniform(0,2*3.14) # theta
      userdata.current_goal={"name":"random","x":rand_x,"y":rand_y,"theta":rand_theta}
      return 'selected'

class FakeState(smach.State):
  def __init__(self):
    smach.State.__init__(self,
        outcomes=['set','failed'],
        input_keys=['script_time','current_goal','position_last_seen','person_name'],
        output_keys=['script_time','current_goal','person_name','position_last_seen'])

  def execute(self,userdata):
    sf = ScreenFormat("FakeState")
    pos={"name":"fake","x":1.5,"y":-1.5,"theta":0.0}
    userdata.position_last_seen=pos
    userdata.person_name="Caro"
    userdata.current_goal=pos
    #userdata.script_time=3
    return 'set'

class GoToGoal_aided(smach.State):
  def __init__(self):
    smach.State.__init__(self,
        outcomes=['approached_goal','update_goal','failed'],
        input_keys=[ 'tl','search_while_moving','current_goal','position_last_seen','person_name'],
        output_keys=['tl','search_while_moving','current_goal','position_last_seen','person_detected_at_goal','person_name'])
    rospy.Subscriber(TOPIC_TRACKED_HUMANS,TrackedHumans, self.callback)
    self.detections=list()
    self.utils=Utils()

  def callback(self,msg):
    # go through list of detections and append them to detection list
    if len(msg.trackedHumans) >0:
      #clear detection list
      #del self.detections[:]
      for i in xrange( len(msg.trackedHumans)):
        #print "m------------------------> appending %s."%msg.trackedHumans[i].identity
	#print msg.trackedHumans[i].location.point
	pose_received = Detection()
	pose_received.pose.header = msg.trackedHumans[i].location.header
	pose_received.pose.pose.position = msg.trackedHumans[i].location.point
	transformed_pose=self.utils.transformPose(pose_received.pose,get_transform_listener())
	#print transformed_pose
	msg.trackedHumans[i].location.point = transformed_pose.pose.position
        self.detections.append(msg.trackedHumans[i])
    return

  def check_detections(self,detections,name):
    if len(detections)==0:
      return False
    for det in detections:
      #print "checking %s"%str(det.label)
      if name == str(det.identity):
        print "det.identity=", det.identity
        print "NAME FOUND"
        # when name found  reset detection list
        del self.detections[:]
        return det
    return False


  def execute(self,userdata):
    sf = ScreenFormat("GoToGoal_aided")
    if userdata.search_while_moving==False:
      print "moving towards goal without searching"
      stop_base=False

      pose=list()
      pose.append(float(userdata.current_goal.x))
      pose.append(float(userdata.current_goal.y))
      pose.append(float(userdata.current_goal.theta))
      handle_base = sss.move("base", pose,blocking=False)

      while not rospy.is_shutdown() and stop_base==False :
        # check if goal has been approached close enough
        if self.utils.goal_approached(userdata.current_goal,get_transform_listener(),dist_threshold=APPROACHED_THRESHOLD):
          sss.stop("base")
          stop_base=True
        rospy.sleep(0.3)
      userdata.person_detected_at_goal=False
      return 'approached_goal'



    elif userdata.search_while_moving==True:
      print "moving towards goal while searching"
      # try reaching pose
      if DOUBLECHECK==True:
        userdata.person_detected_at_goal=False

      # delete old detections
      del self.detections[:]
      print "Length detections %i"%len(self.detections)
      pose=list()
      pose.append(float(userdata.current_goal.x))
      pose.append(float(userdata.current_goal.y))
      pose.append(float(userdata.current_goal.theta))
      handle_base = sss.move("base", pose,blocking=False)

    ## init variables
    #stopping_time = 0.0
    #announce_time = 0.0
    #freq = 2.0 # Hz
    #yellow = False


      # check for goal status
      stop_base=False
      while not rospy.is_shutdown() and stop_base==False :
        print "looping.."
        time.sleep(1)
        detection=self.check_detections(self.detections,userdata.person_name)
        # check if goal update is necessary
        if detection is not False:

          msg_pos=detection.location.point
          #det_pos={"name":"det","x":msg_pos.x,"y":msg_pos.y,"theta":0.0}
          det_pos=Pose2D()
          det_pos.x=msg_pos.x
          det_pos.x=msg_pos.y
          det_pos.theta=0.0
          # confirm detection of person at goal
          userdata.person_detected_at_goal=True
          if self.utils.update_goal(userdata.current_goal,det_pos):
            print "updating goal"
            userdata.current_goal=det_pos
            userdata.position_last_seen=det_pos
            print "stop base"
            sss.stop("base")
            stop_base=True
            return 'update_goal'
          else:
            print "goal similar to current goal"

        # check if goal has been approached close enough
        if self.utils.goal_approached(userdata.current_goal,get_transform_listener(),dist_threshold=APPROACHED_THRESHOLD):
          sss.stop("base")
          stop_base=True
          return 'approached_goal'
        else:
         print "not close enough to person"

      return 'failed'

  #	# finished with succeeded
  #	if (handle_base.get_state() == 3):
  #		sss.set_light('green')
  #		return 'reached'
  #	# finished with aborted
  #	elif (handle_base.get_state() == 4):
  #		sss.set_light('green')
  #		sss.stop("base")
  #		return 'not_reached'
  #	# finished with preempted or canceled
  #	elif (handle_base.get_state() == 2) or (handle_base.get_state() == 8):
  #		sss.set_light('green')
  #		sss.stop("base")
  #		return 'not_reached'
  #	# return with error
  #	elif (handle_base.get_error_code() > 0):
  #		print "error_code = " + str(handle_base.get_error_code())
  #		sss.set_light('red')
  #		return 'failed'

  #	# check if the base is moving
  #	loop_rate = rospy.Rate(freq) # hz
  #	if not self.is_moving: # robot stands still			
  #		# increase timers
  #		stopping_time += 1.0/freq
  #		announce_time += 1.0/freq

  #		# abort after timeout is reached
  #		if stopping_time >= self.timeout:
  #			sss.set_light('green')
  #			sss.stop("base")
  #			return 'not_reached'
  #		
  #		# announce warning after every 10 sec
  #		if announce_time >= 10.0:
  #			sss.say([self.warnings[random.randint(0,len(self.warnings)-1)]],False)
  #			announce_time = 0.0

  #		# set light to "thinking" after not moving for 2 sec
  #		if round(stopping_time) >= 2.0:
  #			sss.set_light("blue")
  #			yellow = False
  #	else:
  #		# robot is moving
  #		if not yellow:
  #			sss.set_light("yellow")
  #			yellow = True
    
    # sleep
    #loop_rate.sleep()






class GoToGoal(smach.State):
  def __init__(self):
    smach.State.__init__(self,
        outcomes=['approached_goal','update_goal','failed'],
        input_keys=[ 'search_while_moving','current_goal','position_last_seen','person_name'],
        output_keys=['search_while_moving','current_goal','position_last_seen','person_detected_at_goal','person_name'])
    rospy.Subscriber(TOPIC_PEOPLE_DETECTION,DetectionArray, self.callback)
    self.detections=list()

    self.utils=Utils()

  def callback(self,msg):
    # go through list of detections and append them to detection list
    if len(msg.detections) >0:
      #clear detection list
      #del self.detections[:]
      for i in xrange( len(msg.detections)):
        self.detections.append(msg.detections[i])
        #print "appending %s."%msg.detections[i].label
    return

  def check_detections(self,detections,name):
    print "CHECKING DETECTIONS for %s"%name
    if len(detections)==0:
      return False
    for det in detections:
      #print "checking %s"%str(det.label)
      if name == str(det.label):
        print "det.label=", det.label
        print "NAME FOUND"
        # when name found  reset detection list
        del self.detections[:]
        return det
    return False

  def execute(self,userdata):
    sf = ScreenFormat("GoToGoal")
    if userdata.search_while_moving==False:
      print "moving towards goal without searching"
      stop_base=False

      pose=list()
      pose.append(float(userdata.current_goal.x))
      pose.append(float(userdata.current_goal.y))
      pose.append(float(userdata.current_goal.theta))
      handle_base = sss.move("base", pose,blocking=False)

      while not rospy.is_shutdown() and stop_base==False :
        if userdata.person_name!=None:
          detection=self.check_detections(self.detections,userdata.person_name)
          detection=False
          # check if goal update is necessary
          if detection is not False:

            print "detection.pose=", detection.pose
            transformed_pose=self.utils.transformPose(detection.pose,get_transform_listener())
            print "transformed_pose=", transformed_pose
            msg_pos=transformed_pose.pose.position

            det_pose=Pose2D()
            det_pose.x=msg_pos.x
            det_pose.y=msg_pos.y
            det_pose.theta=0.0#msg_pos.theta
            userdata.position_last_seen=det_pose
            rospy.loginfo("position last seen has been updated")
          # check if goal has been approached close enough
        if self.utils.goal_approached(userdata.current_goal,get_transform_listener(),dist_threshold=APPROACHED_THRESHOLD):
          sss.stop("base")
          stop_base=True
        rospy.sleep(0.3) 
      userdata.person_detected_at_goal=False
      return 'approached_goal'



    elif userdata.search_while_moving==True:
      print "moving towards goal while searching"
      # try reaching pose
      if DOUBLECHECK==True:
        userdata.person_detected_at_goal=False

      # delete old detections
      del self.detections[:]
      print "Length detections %i"%len(self.detections)
      pose=list()
      pose.append(float(userdata.current_goal.x))
      pose.append(float(userdata.current_goal.y))
      pose.append(float(userdata.current_goal.theta))
      handle_base = sss.move("base", pose,blocking=False)

    ## init variables
    #stopping_time = 0.0
    #announce_time = 0.0
    #freq = 2.0 # Hz
    #yellow = False


      # check for goal status
      stop_base=False
      while not rospy.is_shutdown() and stop_base==False :
        print "looping.."
        time.sleep(1)
        detection=self.check_detections(self.detections,userdata.person_name)
        # check if goal update is necessary
        if detection is not False:

          transformed_pose=self.utils.transformPose(detection.pose,get_transform_listener())
          msg_pos=transformed_pose.pose.position

          det_pose=Pose2D()
          det_pose.x=msg_pos.x
          det_pose.y=msg_pos.y
          det_pose.theta=0.0#msg_pos.theta
          # confirm detection of person at goal
          userdata.person_detected_at_goal=True
          if self.utils.update_goal(userdata.current_goal,det_pos):
            print "updating goal"
            userdata.current_goal=det_pos
            userdata.position_last_seen=det_pos
            print "stop base"
            sss.stop("base")
            stop_base=True
            return 'update_goal'
          else:
            print "goal similar to current goal"

        # check if goal has been approached close enough
        if self.utils.goal_approached(userdata.current_goal,get_transform_listener(),dist_threshold=APPROACHED_THRESHOLD):
          sss.stop("base")
          stop_base=True
          return 'approached_goal'
        else:
         print "not close enough to person"

      return 'failed'

  #	# finished with succeeded
  #	if (handle_base.get_state() == 3):
  #		sss.set_light('green')
  #		return 'reached'
  #	# finished with aborted
  #	elif (handle_base.get_state() == 4):
  #		sss.set_light('green')
  #		sss.stop("base")
  #		return 'not_reached'
  #	# finished with preempted or canceled
  #	elif (handle_base.get_state() == 2) or (handle_base.get_state() == 8):
  #		sss.set_light('green')
  #		sss.stop("base")
  #		return 'not_reached'
  #	# return with error
  #	elif (handle_base.get_error_code() > 0):
  #		print "error_code = " + str(handle_base.get_error_code())
  #		sss.set_light('red')
  #		return 'failed'

  #	# check if the base is moving
  #	loop_rate = rospy.Rate(freq) # hz
  #	if not self.is_moving: # robot stands still			
  #		# increase timers
  #		stopping_time += 1.0/freq
  #		announce_time += 1.0/freq

  #		# abort after timeout is reached
  #		if stopping_time >= self.timeout:
  #			sss.set_light('green')
  #			sss.stop("base")
  #			return 'not_reached'
  #		
  #		# announce warning after every 10 sec
  #		if announce_time >= 10.0:
  #			sss.say([self.warnings[random.randint(0,len(self.warnings)-1)]],False)
  #			announce_time = 0.0

  #		# set light to "thinking" after not moving for 2 sec
  #		if round(stopping_time) >= 2.0:
  #			sss.set_light("blue")
  #			yellow = False
  #	else:
  #		# robot is moving
  #		if not yellow:
  #			sss.set_light("yellow")
  #			yellow = True
    
    # sleep
    #loop_rate.sleep()

class Observe_aided(smach.State):
  def __init__(self):
    smach.State.__init__(self,
      outcomes=['detected','not_detected','failed'],
      input_keys= ['tl', 'person_detected_at_goal','rotate_while_observing','person_name','predefined_goals','current_goal','position_last_seen','script_time'],
      output_keys=['tl','person_detected_at_goal','rotate_while_observing','person_name','predefined_goals','current_goal','position_last_seen','script_time'])
    rospy.Subscriber(TOPIC_TRACKED_HUMANS,DetectionArray, self.callback)
    self.detections=      list()
    self.false_detections=list()
    self.rep_ctr=0
    self.utils=Utils()

  def callback(self,msg):
    # go through list of detections and append them to detection list
    if len(msg.trackedHumans) >0:
      #clear detection list
      #del self.detections[:]
      for i in xrange( len(msg.trackedHumans)):
        #print "o------------------------> appending %s."%msg.trackedHumans[i].identity
	#print msg.trackedHumans[i].location.point
	pose_received = Detection()
	pose_received.pose.header = msg.trackedHumans[i].location.header
	pose_received.pose.pose.position = msg.trackedHumans[i].location.point
	transformed_pose=self.utils.transformPose(pose_received.pose,get_transform_listener())
	#print transformed_pose
	msg.trackedHumans[i].location.point = transformed_pose.pose.position
        self.detections.append(msg.trackedHumans[i])
    return

  def execute(self, userdata):
    sf = ScreenFormat("Observe_aided")
    if userdata.rotate_while_observing==False:
      # give person time to order and observe in the meantime
      rospy.sleep(2)

      # pick detection label which is majority in list
      #TODO pick first is temporary hack

      if len(self.detections)==0:
        userdata.person_detected_at_goal=False
        return 'not_detected'
      else:
        det=self.utils.extract_detection(self.detections)
        if(userdata.person_name)==None:
          userdata.person_name=det.label
        msg_pos=det.location.point
        det_pos=Pose2D()
        det_pos.x=msg_pos.x
        det_pos.x=msg_pos.y
        det_pos.theta=0.0
        userdata.position_last_seen=det_pos
        #userdata.current_goal=userdata.predefined_goals["kitchen"]
        #sss.say(['Thank you for your order, %s.'%str(self.detections[0].label)])
        userdata.person_detected_at_goal=True
        return 'detected'
    elif userdata.rotate_while_observing==True:
      del self.detections[:]
      rel_pose=list()
      rel_pose.append(0)
      rel_pose.append(0)
      rel_pose.append(0.1)
      for i in xrange(70):
        handle_base = sss.move_base_rel("base", rel_pose,blocking =True)
        det=self.utils.extract_detection(self.detections)
        del self.detections[:]
        if det !=False:
            print "person detected while rotating"
            # stop observation
            sss.stop("base")
            # update goal
            msg_pos=det.location.point
            det_pos=Pose2D()
            det_pos.x=msg_pos.x
            det_pos.x=msg_pos.y
            det_pos.theta=0.0
            userdata.position_last_seen=det_pos
            if (userdata.person_name)==None:
              userdata.person_name=det.label
            userdata.person_detected_at_goal=True
            return 'detected'
        else:
            print "person not found at goal"
            userdata.person_detected_at_goal=False
            #return 'not_detected'

      return 'not_detected'



class Observe(smach.State):
  def __init__(self):
    smach.State.__init__(self,
      outcomes=['detected','not_detected','failed'],
      input_keys=[ 'tl','person_detected_at_goal','rotate_while_observing','person_name','predefined_goals','current_goal','position_last_seen','script_time'],
      output_keys=['tl','person_detected_at_goal','rotate_while_observing','person_name','predefined_goals','current_goal','position_last_seen','script_time'])
    rospy.Subscriber(TOPIC_PEOPLE_DETECTION,DetectionArray, self.callback)
    self.detections=      list()
    self.false_detections=list()
    self.rep_ctr=0
    self.utils=Utils()
    #self.tf = TransformListener()

  def callback(self,msg):
    # go through list of detections and append them to detection list
    if len(msg.detections) >0:
      #clear detection list
      #del self.detections[:]
      for i in xrange( len(msg.detections)):
        self.detections.append(msg.detections[i])
    return

  def execute(self, userdata):
    sf = ScreenFormat("Observe")
    if userdata.rotate_while_observing==False:
      # give person time to order and observe in the meantime
      rospy.sleep(2)

      # pick detection label which is majority in list
      #TODO pick first is temporary hack

      if len(self.detections)==0:
        userdata.person_detected_at_goal=False
        return 'not_detected'
      else:
        det=self.utils.extract_detection(self.detections)
        if(userdata.person_name)==None:
          userdata.person_name=det.label

        transformed_pose=self.utils.transformPose(det.pose,get_transform_listener())

        msg_pos=transformed_pose.pose.position
        det_pos=Pose2D()
        det_pos.x=msg_pos.x
        det_pos.y=msg_pos.y
        det_pos.theta=0.0

        userdata.position_last_seen=det_pos
        print "observe: userdata.position_last_seen=", userdata.position_last_seen

        #userdata.current_goal=userdata.predefined_goals["kitchen"]
        #sss.say(['Thank you for your order, %s.'%str(self.detections[0].label)])
        userdata.person_detected_at_goal=True
        return 'detected'
    elif userdata.rotate_while_observing==True:
      del self.detections[:]
      rel_pose=list()
      rel_pose.append(0)
      rel_pose.append(0)
      rel_pose.append(-0.1)
      for i in xrange(80):
        if i==5:
          rel_pose.pop()
          rel_pose.append(0.1)
        handle_base = sss.move_base_rel("base", rel_pose,blocking =True)
        det=self.utils.extract_detection(self.detections)
        del self.detections[:]
        if det !=False:
            print "person detected while rotating"
            # stop observation
            sss.stop("base")
            # update goal
            #msg_pos=det.pose.pose.position
            transformed_pose=self.utils.transformPose(det.pose,get_transform_listener())

            msg_pos=transformed_pose.pose.position

            det_pos=Pose2D()
            det_pos.x=msg_pos.x
            det_pos.y=msg_pos.y
            det_pos.theta=0.0
            userdata.position_last_seen=det_pos
            print "observe rot: userdata.position_last_seen=", userdata.position_last_seen
            if (userdata.person_name)==None:
              userdata.person_name=det.label
            userdata.person_detected_at_goal=True
            return 'detected'
        else:
            print "person not found at goal"
            userdata.person_detected_at_goal=False
            #return 'not_detected'

      return 'not_detected'



class MoveBaseTemp(smach.State):
    def __init__(self):
      smach.State.__init__(self,
        input_keys=['current_goal'],
        outcomes=['reached','failed'])

    def execute(self,userdata):
      sf = ScreenFormat("MoveBaseTemp")
      target_pose=list()
      if userdata.current_goal['name'] is "kitchen":
        target_pose.append(0.1)
        target_pose.append(0)
        target_pose.append(0)
      elif userdata.current_goal['name'] is "couch":
        target_pose.append(-0.1)
        target_pose.append(0)
        target_pose.append(0)
      else:
        target_pose.append(-0.1)
        target_pose.append(-0.1)
        target_pose.append(0)


      for step in xrange(2):
        handle_base = sss.move_base_rel("base", target_pose)
      return 'reached'




#class Rotate(smach.State):
#  #class handles the rotation until program is stopped
#  def __init__(self):
#    self.tf = TransformListener()
#    smach.State.__init__(self,
#      outcomes=['finished','failed'],
#      input_keys=['base_pose','stop_rotating','person_id'],
#      output_keys=['detected'])
#    rospy.Subscriber("/cob_people_detection/detection_tracker/face_position_array",DetectionArray, self.callback)
#    self.stop_rotating=False
#    self.detections=      list()
#    self.false_detections=list()
#
#  def callback(self,msg):
#    # go through list of detections and append them to detection list
#    if len(msg.detections) >0:
#      #clear detection list
#      del self.detections[:]
#      for i in xrange( len(msg.detections)):
#        self.detections.append(msg.detections[i].label)
#    return
#
#  def execute(self, userdata):
#    print "ACCOMPANY-> ROTATE"
#    sss.say(["I am going to take a look around now."])
#
#    # get position from tf
#    if self.tf.frameExists("/base_link") and self.tf.frameExists("/map"):
#        t = self.tf.getLatestCommonTime("/base_link", "/map")
#        position, quaternion = self.tf.lookupTransform("/base_link", "/map", t)
#  # calculate angles from quaternion
#	[r,p,y]=euler_from_quaternion(quaternion)
#	#print r
#	#print p
#	#print y
#  #print position
#    else:
#        print "No transform available"
#        return "failed"
#
#    time.sleep(1)
#    self.stop_rotating=False
#    # create relative pose - x,y,theta
#    curr_pose=list()
#    curr_pose.append(0)
#    curr_pose.append(0)
#    curr_pose.append(0.1)
#
#    while not rospy.is_shutdown() and self.stop_rotating==False and curr_pose[2]< 3.14:
#      handle_base = sss.move_base_rel("base", curr_pose)
#
#      #check in detection and react appropriately
#      for det in self.detections:
#        # right person is detected
#        if det == userdata.id:
#          self.stop_rotating=True
#          sss.say(['I have found you, %s! Nice to see you.'%str(det)])
#        elif det in self.false_detections:
#        # false person is detected
#          print "Already in false detections"
#       #  person detected is unknown - only react the first time
#        elif det == "Unknown":
#          print "Unknown face detected"
#          sss.say(['Hi! Nice to meet you, but I am still searching for %s.'%str(userdata.id)])
#          self.false_detections.append("Unknown")
#      # wrong face is detected the first time
#        else:
#          self.false_detections.append(det)
#          print "known - wrong face detected"
#          sss.say(['Hello %s! Have you seen %s.'%(str(det),str(userdata.id))])
#      #clear detection list, so it is not checked twice

#      del self.detections[:]
#      time.sleep(2)
#
#    print "-->stop rotating"
#    return 'finished'

#class Talk(smach.State):
#  def __init__(self):
#    smach.State.__init__(self,
#      outcomes=['found','not_found','failed'],
#      input_keys=['id','detected']
#      )
#
#    self.phrases=[
#        "I am looking for ",
#        "Hello, nice to meet you! I don't know you yet!",
#        ", nice to see you again",
#        "No,you are not "
#        ]
#
#  def execute(self, userdata):
#    name= str(userdata.id)
#    print "wanted: %s"%name
#    print "found:  %s"% userdata.detected
#    if userdata.id != userdata.detected:
#      #speech=self.phrases[3]+name+" !"
#      sss.say(['No, I am sorry, but you are not %s.'%str(name)])
#      return 'not_found'
#    else:
#      sss.say(['I have found you, %s! Nice to see you.'%str(name)])
#      time.sleep(2)
#      return 'found'


############### SCHEDULER #######################
# this class controls the scr
class Scheduler(smach.State):
  def __init__(self):
    smach.State.__init__(self,
      outcomes=['e0_success','e1_success','e2_success','e3_success','e4_success','e5_failure','failed'],
      #input_keys= ['predefinitions','callback_config'],
      input_keys=['predefinitions','callback_config','position_last_seen','person_name','person_detected_at_goal','current_goal','search_while_moving','rotate_while_observing'],
      output_keys=['predefinitions','callback_config','position_last_seen','person_name','person_detected_at_goal','current_goal','use_perimeter_goal','search_while_moving','rotate_while_observing'])
    self.e=0
    self.failure_ctr=0
    self.predefined_goals=PREDEFINED_GOALS
    self.utils=Utils()

    # define script script events
    # e=0 ---> set start goal1 command going there without observing
    #
    # e=1 ---> begin observing goal1  without rotating
    #
    # e=2 ---> when detection has been made go to goal2 - when detections has
    # not been made return to e=1 with roating
    #
    # e=3 ---> wait at goal2 then proceed to Search
    #
    # e=4 ---> check if person was found at goal. If not initiate rotation
    # observation
    #

  def execute(self, userdata):
    sf = ScreenFormat("Scheduler")
    #feed predefined stuff  to userdata
    # event 0#################################
    if self.e==0:
      userdata.position_last_seen=None
      userdata.person_name=None
      userdata.person_detected_at_goal=False
      userdata.rotate_while_observing=False
      print userdata.callback_config
      #userdata.current_goal=self.predefined_goals["kitchen"]
      userdata.current_goal=self.predefined_goals["couch"]
      userdata.search_while_moving=False
      # increment event
      self.e=1
      rospy.loginfo( ">>>>>>>>>>>>>>>>>>>>> event 0 success -> going to couch")
      return 'e0_success'

    # event 1#################################
    elif self.e==1:
      userdata.rotate_while_observing=False
      # increment event
      self.e=2
      rospy.loginfo( ">>>>>>>>>>>>>>>>>>>>> event 1 success -> observing")
      return 'e1_success'

    # event 2#################################
    elif self.e==2:
      if userdata.person_detected_at_goal==True:
        userdata.current_goal=self.predefined_goals["kitchen"]
        #userdata.current_goal=self.predefined_goals["couch"]
        userdata.search_while_moving=False
        print "e2: userdata.position_last_seen=", userdata.position_last_seen
        # increment event
        self.e=3
        sss.say(["Thank you for your order %s. I am going to fetch it."%str(userdata.person_name)])
        rospy.loginfo( ">>>>>>>>>>>>>>>>>>>>> event 2 success -> going to kitchen")
        return 'e2_success'
      elif userdata.person_detected_at_goal==False:
        #check couter so it doesn't get stuck here without finding anyone
        if self.failure_ctr>=2:
          rospy.logwarn( "after observation target could still not be found")
          sss.say(["I found nobody to take an order from. What a pity"])
          return 'failed'
        userdata.rotate_while_observing= True
        # do not increment event ->repeat step
        rospy.loginfo( ">>>>>>>>>>>>>>>>>>>>> event 2 failure -> trying with rotation")
        self.failure_ctr+=1
        return 'e1_success'

    # event 3#################################
    elif self.e==3:
#      userdata.current_goal=userdata.position_last_seen	# hack: does not work for goals within obstacles
      userdata.current_goal=self.predefined_goals["couch"]
      userdata.use_perimeter_goal = False
      #userdata.current_goal=self.predefined_goals["middle"]
      #userdata.person_name="Caro"
      print "e3: userdata.position_last_seen=", userdata.position_last_seen
#     hack: because navigation is imcapable, the following lines are commented out
#      if userdata.position_last_seen!=None:
#        userdata.current_goal=userdata.position_last_seen
#        userdata.use_perimeter_goal = True
      print "userdata.current_goal=", userdata.current_goal
      time.sleep(0.1)
      userdata.search_while_moving=True
      self.e=4
      sss.say(["I am coming back to you %s."%str(userdata.person_name)])
      rospy.loginfo( ">>>>>>>>>>>>>>>>>>>>> event 3 success -> done job in kitchen  - searching begins")
      return 'e3_success'
    # event 4#################################
    elif self.e==4:
      if userdata.person_detected_at_goal==True:
        print "say it e4: Here is your order."
        sss.say(["Here is your order."])
        rospy.loginfo( ">>>>>>>>>>>>>>>>>>>>> event 4 success -> approached position of target person - finished")
        return 'e4_success'
      elif userdata.person_detected_at_goal==False:
        userdata.rotate_while_observing= True
        userdata.use_perimeter_goal = True
        self.e=5
        rospy.loginfo( ">>>>>>>>>>>>>>>>>>>>> event 4 failure -> approached position but didnt find target person - rotating observation")
        return 'e3_success'     # 'e1_success'
    # event 5#################################
    elif self.e==5:
      if userdata.person_detected_at_goal==True:
        userdata.search_while_moving=True
        # decrement to event 4
        self.e=4
        if self.utils.update_goal(userdata.position_last_seen,userdata.current_goal)==True:
          return 'e4_success'
        else:
          rospy.loginfo( ">>>>>>>>>>>>>>>>>>>>> event 5 success -> going to new target position")
          userdata.use_perimeter_goal = True
          return 'e3_success'
      elif userdata.person_detected_at_goal==False:
        userdata.search_while_moving=True
        userdata.rotate_while_observing= True
        # decrement to event 4
        self.e=4
        sss.say(["Where are you? I will search randomly now."])
        rospy.loginfo( ">>>>>>>>>>>>>>>>>>>>> event 5 success -> now searching random positions")
        return 'e5_failure'

    else:
      rospy.logerr( " FAILURE - not handled event")



class Seek_aided_generic(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,
                outcomes=['finished','failed'],
                input_keys=['predefinitions','callback_config','position_last_seen','person_name','person_detected_at_goal','current_goal','search_while_moving','rotate_while_observing'],
                output_keys=['predefinitions','callback_config','position_last_seen','person_name','person_detected_at_goal','current_goal','search_while_moving','rotate_while_observing'])
        with self:
            smach.StateMachine.add('SCHEDULER',Scheduler(),
                                   transitions={'e0_success':'GOTOGOAL',
                                                'e1_success':'OBSERVE',
                                                'e2_success':'GOTOGOAL',
                                                'e3_success':'SEARCH',
                                                'e4_success':'finished',  # 'failed', # should not occur
                                                'e5_failure':'SELECT_RANDOM_GOAL', # should not occur
                                                'failed':'failed'})
            smach.StateMachine.add('SELECT_RANDOM_GOAL',SelectRandomGoal(),
                                   transitions={'selected':'GOTOGOAL',
                                                'not_selected':'failed',
                                                'failed':'failed'})
            smach.StateMachine.add('OBSERVE',Observe(),
                                   transitions={'failed':'failed',
                                                'detected':'SCHEDULER',
                                                'not_detected':'SCHEDULER'})
            smach.StateMachine.add('GOTOGOAL',GoToGoal(),
                                   transitions={'failed':'failed',
                                                'update_goal':'GOTOGOAL',
                                                'approached_goal':'SCHEDULER'})
            smach.StateMachine.add("SEARCH",SearchPersonGeneric(),
                                    transitions={'failed':'failed',
                                                  'finished': 'SCHEDULER'}) #'finished'})
class Seek_aided(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,
                outcomes=['finished','failed'])
        with self:
            smach.StateMachine.add('SCHEDULER',Scheduler(),
                                   transitions={'e0_success':'GOTOGOALAIDED',
                                                'e1_success':'OBSERVE',
                                                'e2_success':'GOTOGOALAIDED',
                                                'e3_success':'GOTOGOALAIDED',
                                                'e4_success':'finished',
                                                'e5_failure':'SELECT_RANDOM_GOAL',
                                                'failed':'failed'})
            smach.StateMachine.add('SELECT_RANDOM_GOAL',SelectRandomGoal(),
                                   transitions={'selected':'GOTOGOALAIDED',
                                                'not_selected':'failed',
                                                'failed':'failed'})
            smach.StateMachine.add('OBSERVE',Observe(),
                                   transitions={'failed':'failed',
                                                'detected':'SCHEDULER',
                                                'not_detected':'SCHEDULER'})
            smach.StateMachine.add('GOTOGOALAIDED',GoToGoal_aided(),
                                   transitions={'failed':'failed',
                                                'update_goal':'GOTOGOALAIDED',
                                                'approached_goal':'SCHEDULER'})


class Seek(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,
                outcomes=['finished','failed'])
        with self:
            smach.StateMachine.add('SCHEDULER',Scheduler(),
                                   transitions={'e0_success':'GOTOGOAL',
                                                'e1_success':'OBSERVE',
                                                'e2_success':'GOTOGOAL',
                                                'e3_success':'GOTOGOAL',
                                                'e4_success':'finished',
                                                'e5_failure':'SELECT_RANDOM_GOAL',
                                                'failed':'failed'})
            smach.StateMachine.add('SELECT_RANDOM_GOAL',SelectRandomGoal(),
                                   transitions={'selected':'GOTOGOAL',
                                                'not_selected':'failed',
                                                'failed':'failed'})
            smach.StateMachine.add('OBSERVE',Observe(),
                                   transitions={'failed':'failed',
                                                'detected':'SCHEDULER',
                                                'not_detected':'SCHEDULER'})


class SM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['ended'])
        with self:
            smach.StateMachine.add('STATE',Seek(),
                transitions={'finished':'ended',
                      'failed':'ended'})
class SM_aided(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['ended'])
        with self:
            smach.StateMachine.add('STATE',Seek_aided(),
                transitions={'finished':'ended',
                      'failed':'ended'})
class SM_aided_generic(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['ended'])
        with self:
            smach.StateMachine.add('STATE',Seek_aided_generic(),
                transitions={'finished':'ended',
                      'failed':'ended'})


def start_prompt():
  print "---------------------------------------------------------"
  print "plese start script with one of the follorwing arguments :"
  print "DEPRECATED-DONT'T USE: aided - TrackedHumans topic is used while searching"
  print "DEPRECATED-DONT'T USE: unaided - Face Recognition topic is used while searching"
  print "people_detection - use Face Recognition only for user approach"
  print "tracked_humans - use Face Recognition for recognition but tracked_humans  for user approach"
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

    rospy.init_node('ApproachUser')
    time.sleep(1)

    # DEPRECATED##############################
    if flag=="aided":
      print "ACCOMPANY AIDED"
      sm = SM_aided()
    elif flag=="unaided":
      print "ACCOMPANY UNAIDED"
      sm = SM()
    # DEPRECATED##############################



    elif flag=="people_detection":
      rospy.loginfo("Running approach user with people detections")
      sm = SM_aided_generic()
      sm.userdata.callback_config={"argname_frame":["pose","header","frame_id"],
                    "msg_element":"detections",
                    "argname_label":["label"],
                    "argname_position":["pose","pose","position"],
                    "argname_header":["pose","header"],
                    "topicname":TOPIC_PEOPLE_DETECTION,
                    "msgclass":DetectionArray}


    elif flag=="tracked_humans":
      rospy.loginfo("Running approach user with tracked humans")
      sm = Seek_aided_generic()
      sm.userdata.callback_config={"argname_frame":["location","header","frame_id"],
                    "msg_element":"trackedHumans",
                    "argname_label":["identity"],
                    "argname_position":["location","point"],
                    "argname_header":["location","header"],
                    "topicname":TOPIC_TRACKED_HUMANS,
                    "msgclass":TrackedHumans}
    else:
      start_prompt()

    sm.userdata.predefinitions={"predefined_goals":PREDEFINED_GOALS,
                              "map_bounds":MAP_BOUNDS,
                              "double_check":DOUBLECHECK,
                              "approached_threshold":APPROACHED_THRESHOLD,
                              "similar_goal_threshold":SIMILAR_GOAL_THRESHOLD,
                              "goal_perimeter":GOAL_PERIMETER}
    # dummy initialization
    sm.userdata.search_while_moving=False
    sm.userdata.rotate_while_observing=False
    sm.userdata.person_detected_at_goal=False
    sm.userdata.person_name=None
    sm.userdata.current_goal=None
    sm.userdata.position_last_seen=None


    sis = smach_ros.IntrospectionServer('SM', sm, 'SM')
    sis.start()
    outcome = sm.execute()
    rospy.spin()
    sis.stop()
  except:
    print"exception"
    os._exit(1)
