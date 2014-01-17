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
import history

#--------------------------------------------------------------------------------
class imagetest(script):

  def Initialize(self):

    if not self.sss.parse:
       rospy.loginfo(" *********** This is the imagetest script ************** ")

    if not self.sss.parse:
       rospy.loginfo("Initializing all components...")

    self.sss.init("tray")
    self.sss.init("torso")
    self.sss.init("arm")

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
       if (overallresult):
          image = history.ActionHistory()
          image.addHistoryAsync('imagetest')

    if not self.sss.parse:
       rospy.loginfo("move tray on ::0::Care-O-Bot 3.2 to Raised and wait for completion")

    if (overallresult):
       self.sss.move("tray","up",True)
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
   SCRIPT = imagetest()
   SCRIPT.Start()
#--------------------------------------------------------------------------------
