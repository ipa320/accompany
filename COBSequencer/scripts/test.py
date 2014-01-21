#!/usr/bin/python

### This is an auto-generated script - do not edit ###

import subprocess
import time
import sys
sys.path.append('/home/joe/QTProjects/Core/')
import roslib
roslib.load_manifest('cob_script_server')
import rospy

from simple_script_server import script
from config import *            # contains configuration details for UH Robot House inc. DB login info

import MySQLdb

#--------------------------------------------------------------------------------
class test(script):

  def Initialize(self):

    if not self.sss.parse:
       rospy.loginfo(" *********** This is the test script ************** ")

# get a DB connection

    if not self.sss.parse:
       try:

          self.conn = MySQLdb.connect(server_config['mysql_log_server'],
                                      server_config['mysql_log_user'],
                                      server_config['mysql_log_password'],
                                      server_config['mysql_log_db'])

       except MySQLdb.Error, e:
          print "Error %d: %s" % (e.args[0], e.args[1])
          sys.exit (1)

       rospy.loginfo('MySQL  initialized')

#--------------------------------------------------------------------------------
  def Run(self):

# No rules found!

    overallresult=True;        # no rules thus execute all actions

    if not self.sss.parse:
       rospy.loginfo("move torso on ::0::Care-O-Bot 3.2 to home position")

    if (overallresult):
       self.sss.move("arm","home")  
   #    self.sss.move("arm",[[-2.1572567240035734, -1.9104664691761568, -2.5334780195730255, -1.7853311980377056, -0.072798739390243047, 0.91767934923272776, -1.8876618005378798]])
   #    self.sss.move("arm","folded-to-look_at_table")

       self.sss.move("arm","folded")

  #      self.sss.move("arm","home")
  #     self.sss.move("arm","pregrasp")
  #     self.sss.move("arm","home")

   #    self.sss.move("arm","folded")
   #    self.sss.move("arm","pregrasp")
   #     self.sss.move("arm","folded")

   #    self.sss.move("arm","folded")
   #    self.sss.move("arm","grasp")
   #    self.sss.move("arm","folded")

   #    self.sss.move("arm","folded")
   #    self.sss.move("arm","grasp")
   #    self.sss.move("arm","pregrasp")
   #    self.sss.move("arm","grasp")
   #    self.sss.move("arm","folded")

       self.sss.move("arm","intermediatebacklow")
   
    #    self.sss.move("arm","folded")
    #    self.sss.move("arm","intermediateback")
    #    self.sss.move("arm","intermediatefront")
    #    self.sss.move("arm","overtray")
   
   #     self.sss.move("arm","home")
   #     self.sss.move("arm","intermediatefront")
   #     self.sss.move("arm","overtray")
   #     self.sss.move("arm","tray")
   #     self.sss.move("arm","grasp")

 #      self.sss.move("arm","home")
 #      self.sss.move("arm","grasp")
 #      self.sss.move("arm","home")
 #      self.sss.move("arm","grasp")
 #      self.sss.move("arm","folded")

   #    self.sss.move("arm","grasp-to-tray")
   # //   self.sss.move("arm","waveout")
   # //   self.sss.move("arm","wavein")
   #    self.sss.move("arm","overtray")
   #    self.sss.move("arm","tray-to-folded")
    else:
      if not self.sss.parse:
         rospy.loginfo("Action not executed as rule is invalid!")

    if not self.sss.parse:
      if (overallresult):
        sys.stderr.write('Success\n')
      else:
        sys.stderr.write('Failure\n')

#--------------------------------------------------------------------------------
if __name__ == "__main__":
   SCRIPT = test()
   SCRIPT.Start()
#--------------------------------------------------------------------------------
