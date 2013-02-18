#!/usr/bin/python

### This is an auto-generated script - do not edit ###

import subprocess
import time
import sys
import roslib
roslib.load_manifest('cob_script_server')
import rospy

from simple_script_server import script
from config import *            # contains configuration details for UH Robot House inc. DB login info

import MySQLdb

#--------------------------------------------------------------------------------
class medicineAt5pm(script):

  def Initialize(self):

    if not self.sss.parse:
       rospy.loginfo(" *********** This is the medicineAt5pm script ************** ")

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

    overallresult=checkRules(self);        # check the rule set

    if not self.sss.parse:
       rospy.loginfo("Turn light on ::0::Care-O-Bot 3.2 to  red")

    if (overallresult):
       self.sss.set_light("red")
    else:
      if not self.sss.parse:
         rospy.loginfo("Action not executed as rule is invalid!")

    if not self.sss.parse:
      if (overallresult):
        sys.stderr.write('Success\n')
      else:
        sys.stderr.write('Failure\n')

    if not self.sss.parse:
       rospy.loginfo("move ::0::Care-O-Bot 3.2 to ::14:: Living Room Sofa Area in the Living Room and wait for completion")

    if (overallresult):
       self.sss.move("base",[3.88,1.17,23],True)
    else:
      if not self.sss.parse:
         rospy.loginfo("Action not executed as rule is invalid!")

    if not self.sss.parse:
      if (overallresult):
        sys.stderr.write('Success\n')
      else:
        sys.stderr.write('Failure\n')

    if not self.sss.parse:
       rospy.loginfo("move tray on ::0::Care-O-Bot 3.2 to Raised")

    if (overallresult):
       self.sss.move("tray","up",False)
    else:
      if not self.sss.parse:
         rospy.loginfo("Action not executed as rule is invalid!")

    if not self.sss.parse:
      if (overallresult):
        sys.stderr.write('Success\n')
      else:
        sys.stderr.write('Failure\n')

    if not self.sss.parse:
       rospy.loginfo("move torso on ::0::Care-O-Bot 3.2 to the forward position")

    if (overallresult):
       self.sss.move("torso","front",False)
    else:
      if not self.sss.parse:
         rospy.loginfo("Action not executed as rule is invalid!")

    if not self.sss.parse:
      if (overallresult):
        sys.stderr.write('Success\n')
      else:
        sys.stderr.write('Failure\n')

    if not self.sss.parse:
       rospy.loginfo("Turn light on ::0::Care-O-Bot 3.2 to  green")

    if (overallresult):
       self.sss.set_light("green")
    else:
      if not self.sss.parse:
         rospy.loginfo("Action not executed as rule is invalid!")

    if not self.sss.parse:
      if (overallresult):
        sys.stderr.write('Success\n')
      else:
        sys.stderr.write('Failure\n')

    if not self.sss.parse:
       rospy.loginfo("::0::Care-O-Bot 3.2 says 'It's time for your tablets'")

    if (overallresult):
       self.sss.say(["It's time for your tablets"])
    else:
      if not self.sss.parse:
         rospy.loginfo("Action not executed as rule is invalid!")

    if not self.sss.parse:
      if (overallresult):
        sys.stderr.write('Success\n')
      else:
        sys.stderr.write('Failure\n')

    if not self.sss.parse:
       rospy.loginfo("::0::Care-O-Bot 3.2 GUI,walkToKitchen,returnToHome,wait1minute")

    if (overallresult):
       displayGUI(self)
    else:
      if not self.sss.parse:
         rospy.loginfo("Action not executed as rule is invalid!")

    if not self.sss.parse:
      if (overallresult):
        sys.stderr.write('Success\n')
      else:
        sys.stderr.write('Failure\n')


#--------------------------------------------------------------------------------
def checkRules(self):


   # this queries the database rules, executes each and veryfies final result

    overallresult = False    # assume the rule set doesn't apply

   # this extracts the rules set for this script from the database and checks each rule in turn
   # the rules are and'ed or or'ed based on the rule set

    if not self.sss.parse:

         cursorSequence = self.conn.cursor()

         sqlSequence = "SELECT   *  FROM  ActionRules\
                          WHERE     name = 'medicineAt5pm'\
                            AND     ruleType = 'R'\
                          ORDER BY  ruleOrder"

         # get the sequence from the sql query

         cursorSequence.execute(sqlSequence)

         sequenceRows = cursorSequence.fetchall()

         # for each row execute the rule to see it is valid

         recCount = 0

         try:
           for sequenceRow in sequenceRows:

              recCount = recCount + 1

              ANDORSwitch = ""

              andOr = sequenceRow[4]

              if andOr== 1:
                 ANDORSwitch = "AND"

              if andOr== 2:
                 ANDORSwitch = "OR"

              rospy.loginfo("%s %s",sequenceRow[5],ANDORSwitch)

              # now do each rule

              cursorRule = self.conn.cursor()

              sqlRule = sequenceRow[6];

              cursorRule.execute(sqlRule)

              ruleRows = cursorRule.fetchone()

              # returning data means that the rule is true

              if (ruleRows==None):
                 rospy.loginfo("%s FALSE",sqlRule)
                 result = False
              else:
                 rospy.loginfo("%s TRUE",sqlRule)
                 result = True 

              if (recCount == 1): 
                 overallresult = result
                 prevANDORSwitch = ANDORSwitch
              else:                                  # now AND or OR each row in turn
                 if (prevANDORSwitch == "OR"):
                    overallresult = overallresult or result
                 if (prevANDORSwitch == "AND"):
                    overallresult = overallresult and result
                 if (prevANDORSwitch == ""):
                    overallresult = overallresult and result;

                 prevANDORSwitch = ANDORSwitch

           if (overallresult):                      # final result
             rospy.loginfo("Total Rule Set is VALID! :) ")
           else:
             rospy.loginfo("Total Rule Set is INVALID :( ")

         except MySQLdb.Error, e:
           rospy.loginfo("Error %d: %s" % (e.args[0],e.args[1]))
           sys.exit(1)  

         finally:   
           cursorSequence.close() 

         return overallresult


#--------------------------------------------------------------------------------
def displayGUI(self):

   # this sets up the database for GUI display and waits for a result
   # It then executes a script depending on user response

   if not self.sss.parse:

      cursorSequence = self.conn.cursor()

      sqlSequence = "UPDATE userInterfaceGUI SET guiMsgResult = NULL\
                      WHERE name = 'medicineAt5pm'"

      try:
         cursorSequence.execute(sqlSequence)  # get the sequence from the sql query  
         self.conn.commit()                   # Commit  changes in the database

      except MySQLdb.Error, e:
         rospy.loginfo("Error %d: %s" % (e.args[0],e.args[1]))
         self.rollback()
         self.conn.close()
         sys.exit(1)

      awaitingUserResponse = True
      resultFromUser = 0
      numSeconds = 0

      rospy.loginfo("Waiting for user response...")

      while awaitingUserResponse:

         time.sleep(1)

         numSeconds = numSeconds + 1

         if (numSeconds > 60):
            rospy.loginfo("Error: waited too long for user response - exiting!")
            sys.exit(1)  

         sqlSequence = "SELECT * FROM  userInterfaceGUI WHERE name = 'medicineAt5pm'"
         cursorSequence.execute(sqlSequence)      # get the sequence from the sql query
         sequenceRows = cursorSequence.fetchall()

         try:

          for sequenceRow in sequenceRows:      # for each row execute the rule to see it is valid 

            resultFromUser = sequenceRow[13]

            if (resultFromUser == None): # no response
               break

            awaitingUserResponse = False

            rospy.loginfo("User entered %s",resultFromUser)

            if (resultFromUser == 1):
                proc = subprocess.Popen("/home/joe/QTProjects/COBSequencer/scripts/walkToKitchen.py",shell=True, stderr=subprocess.PIPE)
                return_code = proc.wait()
                for line in proc.stderr:
                  if (line.rstrip() == "Failure"):
                     overallresult = False

            if (resultFromUser == 2):
                proc = subprocess.Popen("/home/joe/QTProjects/COBSequencer/scripts/returnToHome.py",shell=True, stderr=subprocess.PIPE)
                return_code = proc.wait()
                for line in proc.stderr:
                  if (line.rstrip() == "Failure"):
                     overallresult = False

            if (resultFromUser == 3):
                proc = subprocess.Popen("/home/joe/QTProjects/COBSequencer/scripts/wait1minute.py",shell=True, stderr=subprocess.PIPE)
                return_code = proc.wait()
                for line in proc.stderr:
                  if (line.rstrip() == "Failure"):
                     overallresult = False

         except MySQLdb.Error, e:
            rospy.loginfo("Error %d: %s" % (e.args[0],e.args[1]))
            sys.exit(1)
#--------------------------------------------------------------------------------
if __name__ == "__main__":
   SCRIPT = medicineAt5pm()
   SCRIPT.Start()
#--------------------------------------------------------------------------------
