#!/usr/bin/env python

import web
import sys
import os

# for database management
import MySQLdb
import json
import time
import math

#to implement a thread that manage the robot's emotion
import threading
import time
import random

urls =(
'/next','next',
'/next/(.*)','next',
'/current','current',
'current/(.*/','current'
)

class next:
	def POST(self):
		interrupted=0
		paras=web.input(current='-1',task="")
		current_client_status=web.websafe(paras.current)
		current_client_status=current_client_status.strip()
		current_client_task=web.websafe(paras.task)
		current_client_task=current_client_task.strip()
		task=""
		print "Received request of next status... (current: %s)"%current_client_status
		
		#read the db configuration from file
   	 	path = os.getcwd()+"/db_cnf"
   		#print path
    		f= open(path)
    		lines = f.readlines()
    		for line in lines:
			values = line.split()
			if values[0]=="likelihood":
				LIKELIHOOD_TREESHOLD=values[1]
			elif values[0]=="user":
				U=values[1]
			elif values[0]=="password":
				P=values[1]
			elif values[0]=="db_name":
				DATABASE_NAME=values[1]
			elif values[0]=="host":
				HOST=values[1]
	    		else:
				print "error, unreadable line in configuration file"
				break
	    	f.close()

		db= MySQLdb.connect(HOST,U,P,DATABASE_NAME)
		cur= db.cursor()
		sql="Select value from Sensors where sensorId=\'999\';"
		cur.execute(sql)
		flag=0
		if cur.rowcount==0:
			a= "error"
			return a
		else:
			reslist=cur.fetchone()
			a=str(reslist[0])
			flag=reslist[0]
		cur= db.cursor()
		sql="Select actionName from RobotActionsHistory order by timestamp DESC limit 1;"
		cur.execute(sql)
		if cur.rowcount==0:
			a= "error"
			return a
		else:
			task="%s"%cur.fetchone()[0]	
		time.sleep(0.25)
		count = 60  #rough timeout of about 15sec --> if nothing changes in 15 sec the response will be empty
		while ((str(flag)==current_client_status) and (task==current_client_task) and interrupted==0
			and count>=0):
			try:
				#print "inside while flag: %s - cur: %s"%(str(flag),current_client_status)
				#print "                   -%s- - task: -%s-"%(task,current_client_task)
				db= MySQLdb.connect(HOST,U,P,DATABASE_NAME)
				cur= db.cursor()
				sql=('SELECT value FROM Sensors WHERE sensorId=\'999\'')
				cur.execute(sql);
				if cur.rowcount==0: 
					a="error"
					flag=1;
					print 'Problem! rowcount=0'
					interrupted=1
					return a
				else:	
					reslist=cur.fetchone()
					flag=reslist[0]
					a=str(reslist[0])
					#print 'res:  %s'%a
					cur= db.cursor()
					sql="Select actionName from RobotActionsHistory order by timestamp DESC limit 1;"
					cur.execute(sql)
					if cur.rowcount==0:
						a= "error"
						return a
					else:
						task="%s"%cur.fetchone()[0]
				time.sleep(0.25)
				count= count - 1 
			except KeyboardInterrupt:
				interrupted=1
				print "ctrl-C received"
				return "-1"
		#print "outside while flag: :%s: - cur: :%s:"%(str(flag),current_client_status)
		if ((str(flag)==current_client_status) and (task==current_client_task)):
			return ""
		else:
			a+=",%s"%task
			return a
		#return a

class current:
	def POST(self):
		print "Received request of current status..."
		
		#read the db configuration from file
   	 	path = os.getcwd()+"/db_cnf"
   		#print path
    		f= open(path)
    		lines = f.readlines()
    		for line in lines:
			values = line.split()
			if values[0]=="likelihood":
				LIKELIHOOD_TREESHOLD=values[1]
			elif values[0]=="user":
				U=values[1]
			elif values[0]=="password":
				P=values[1]
			elif values[0]=="db_name":
				DATABASE_NAME=values[1]
			elif values[0]=="host":
				HOST=values[1]
	    		else:
				print "error, unreadable line in configuration file"
				break
	    	f.close()

		db= MySQLdb.connect(HOST,U,P,DATABASE_NAME)
		cur= db.cursor()
		sql="Select value from Sensors where sensorId=\'999\';"
		cur.execute(sql)
		if cur.rowcount==0:
			a= "error"
			return a
		else:
			a="%s"%cur.fetchone()[0]
			#db= MySQLdb.connect(HOST,U,P,DATABASE_NAME)
			cur= db.cursor()
			sql="Select actionName from RobotActionsHistory order by timestamp DESC limit 1;"
			cur.execute(sql)
			if cur.rowcount==0:
				a= "error"
				return a
			else:
				a+=",%s"%cur.fetchone()[0]
				return a
	
########################################################################################################################################
####################################################     Parte di threading     ########################################################
########################################################################################################################################		

#expression thread
class myExpressionThread (threading.Thread):

	def __init__(self,threadId,name):
		threading.Thread.__init__(self)
		self.threadId=threadId
		self.name=name
		self._stop= threading.Event()

	def run(self):
		delay=0
		#read the db configuration from file
   	 	path = os.getcwd()+"/db_cnf"
   		#print path
    		f= open(path)
    		lines = f.readlines()
    		for line in lines:
			values = line.split()
			if values[0]=="likelihood":
				like=values[1]
			elif values[0]=="user":
				usr=values[1]
			elif values[0]=="password":
				pwd=values[1]
			elif values[0]=="db_name":
				db_name=values[1]
			elif values[0]=="host":
				host=values[1]
	    		else:
				print "error, unreadable line in configuration file"
				break
	    	f.close()
		delay=random.randrange(30,121)
		while (self._stop.isSet()==False):
			time.sleep(delay)
			db= MySQLdb.connect(host,usr,pwd,db_name)
			cur= db.cursor()
			sql="Select id from GUIexpression where ison=\'1\' limit 1;"
			cur.execute(sql)
			if cur.rowcount==0:
				a= "error"
			else:
				a="%s"%cur.fetchone()[0]

			# define here how to change expressions!
			#new= int(a)
			#new=new+1
			#if (new>4):
			#	new=1
			new = random.randrange(1,5)
			delay=random.randrange(30,121)

			db= MySQLdb.connect(host,usr,pwd,db_name)
			cur= db.cursor()
			sql = ('UPDATE GUIexpression SET ison=\'0\' WHERE ison=\'1\'')		
			cur.execute(sql);
			db= MySQLdb.connect(host,usr,pwd,db_name)
			cur= db.cursor()
			sql = ('UPDATE GUIexpression SET ison=\'1\' WHERE id=\'%s\''%new)		
			cur.execute(sql);
			
	def stop(self):
		self._stop.set()

#likelihood thread	
class myLikelihoodThread (threading.Thread):

	def __init__(self,threadId,name):
		threading.Thread.__init__(self)
		self.threadId=threadId
		self.name=name
		self._stop= threading.Event()

	def run(self):
		delay=2
		#read the db configuration from file
   	 	path = os.getcwd()+"/db_cnf"
   		#print path
    		f= open(path)
    		lines = f.readlines()
    		for line in lines:
			values = line.split()
			if values[0]=="likelihood":
				like=values[1]
			elif values[0]=="user":
				usr=values[1]
			elif values[0]=="password":
				pwd=values[1]
			elif values[0]=="db_name":
				db_name=values[1]
			elif values[0]=="host":
				host=values[1]
	    		else:
				print "error, unreadable line in configuration file"
				break
	    	f.close()
		while (self._stop.isSet()==False):
			time.sleep(delay)
			
			
	def stop(self):
		self._stop.set()	

########################################################################################################################################
#############################################################   Main   #################################################################
########################################################################################################################################

if __name__ == "__main__":
	
	expressionThread = myExpressionThread(1,"ExpressionThread")
	expressionThread.start()
	likelihoodThread = myLikelihoodThread(2,"LikelihoodThread")
	likelihoodThread.start()
 	
	app = web.application(urls, globals())
	print "running..."
	app.run()      
	print "app stopped"
	expressionThread.stop()
	likelihoodThread.stop()
	expressionThread.join()
	likelihoodThread.join()
	print "thread stopped. exit."
