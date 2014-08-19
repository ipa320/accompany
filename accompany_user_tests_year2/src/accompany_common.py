#!/usr/bin/python

import roslib
roslib.load_manifest('accompany_user_tests_year2')
import rospy
import os
import threading
import tf

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
