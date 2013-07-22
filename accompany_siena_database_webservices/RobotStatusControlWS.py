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
from datetime import datetime
from datetime import timedelta
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

	def run_old(self):
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

	def run(self):

		sensorsIds = {'toiletFlush':'14','dorbell':'59', 'door':'23'}  
		expressionId = {'basic':1,'sadness':2,'fear':3,'disgust':4,'surprise':5,'anger':6,'joy':7,'low_batteries':8,'squeeze':99} 		

		delay=0.5
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
			scores = { 'basic':0.5, 'disgust':0.0, 'fear':0.0, 'sadness':0.0, 'surprise':0.0, 'joy':0.0, 'lowbatteries':0.0, 'anger':0.0, 'squeeze':0.0 }
			db= MySQLdb.connect(host,usr,pwd,db_name)

			# define here how to change expressions!
			now = datetime.now()
			
			#TIME MANAGEMENT 
			# depending on the hour we can improve slightly the sleep (lowbattery), sadness/joy and others expressions

			cur= db.cursor()   #WATERTOILET EVENT
			sql = ('SELECT lastTimeActive FROM Sensors WHERE sensorId=\'%s\''%sensorsIds['toiletFlush'])	
			cur.execute(sql);
			if cur.rowcount>0:
				watertoiletflushtime=cur.fetchone()[0]
				#print watertoiletflushtime 
				#lastflush= datetime.fromtimestamp(watertoiletflushtime)
				lastflush=watertoiletflushtime
				tdelta= now-lastflush
				seconds=(tdelta.microseconds + ( tdelta.seconds + tdelta.days * 24 *3600 )* 10**6) / 10**6
				#print seconds
				if (seconds<30):    # IF WATER WAS FLUSHED IN THE LAST 30 SECONDS DISGUST SCORES INCREASED BY 1
					scores['disgust']+=1.5

			cur= db.cursor()    #DORBELL EVENT & open door
			sql = ('SELECT lastTimeActive FROM Sensors WHERE sensorId=\'%s\''%sensorsIds['dorbell'])	
			cur.execute(sql);
			if cur.rowcount>0:
				ringtime=cur.fetchone()[0]
				tdelta= now-ringtime
				seconds=(tdelta.microseconds + ( tdelta.seconds + tdelta.days * 24 *3600 )* 10**6) / 10**6
				if (seconds<30):    # IF DOORBELLM RIUNGED IN LAST 30 SECONDS SURPRISED INCREASED BY 2.5
					scores['surprise']+=2.5 #surprise
			sql = ('SELECT lastTimeActive FROM Sensors WHERE sensorId=\'%s\''%sensorsIds['door'])	#cannot check door open -> check goal setted
			cur.execute(sql);
			if cur.rowcount>0:
				ringtime=cur.fetchone()[0]
				tdelta= now-ringtime
				secondsdoor=(tdelta.microseconds + ( tdelta.seconds + tdelta.days * 24 *3600 )* 10**6) / 10**6
				if (secondsdoor < 60 and secondsdoor < seconds):    # IF doorbell was answerd in last minute & it's sooner than last doorbell ring
					scores['surprise']-=2.5 #surprise
					scores['joy']+=2

			cur=db.cursor()    #SQUEEZE EVENTS
			sql = ('SELECT value FROM SqueezeHistory WHERE timestamp > (NOW()-30);')
			cur.execute(sql)
				#print "rowcount", cur.rowcount
				#if cur.rowcount==2:
				#	scores['sadness']+=1.5 #surprise
				#elif cur.rowcount>3: 
			if cur.rowcount>0:
				scores['squeeze']+=10.0

			cur = db.cursor()   #USER COMMANDS EVENTS
			#sql = ('SELECT id FROM RobotActionsHistory WHERE timestamp > (NOW()-1800)')   #180 - last half hour doing nothing -->sad
			#cur.execute(sql)
			#if cur.rowcount==0:
			#	scores['sadness']+=1.0

			#MAX SCORE SELECTED AS NEW EXPRESSION
			mymax=scores['basic']
			new='basic'
			#print 0, new, mymax 
			for  i, v in enumerate(['disgust','fear','sadness','surprise','joy','lowbatteries','anger','squeeze']):
				#print i, v, scores[v]
				if (scores[v]>mymax):
					mymax=scores[v]
					new=v	
	
			#print "-> ", new
			db= MySQLdb.connect(host,usr,pwd,db_name)
			cur= db.cursor()
			sql = ('UPDATE GUIexpression SET ison=\'1\' WHERE id=\'%s\''%expressionId[new])	
			cur.execute(sql);
			db= MySQLdb.connect(host,usr,pwd,db_name)
			cur= db.cursor()		
			sql = ('UPDATE GUIexpression SET ison=\'0\' WHERE id<>\'%s\''%expressionId[new])	
			cur.execute(sql);
			#print new;
			time.sleep(delay)
			
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
			#LIKELIHOODS COMPUTATION	
			# by now it's "hardcoded", it will be defined a way to specify likelihood computation rules
			
			#coffee compuatation
			
			#water computation

			#lights on computation
			
			#lights off computation

			time.sleep(delay)
			
			
	def stop(self):
		self._stop.set()	

########################################################################################################################################
#############################################################   Main   #################################################################
########################################################################################################################################

if __name__ == "__main__":
	
	expressionThread = myExpressionThread(1,"ExpressionThread")
	expressionThread.start()
	#likelihoodThread = myLikelihoodThread(2,"LikelihoodThread")
	#likelihoodThread.start()
 	
	app = web.application(urls, globals())
	print "running..."
	app.run()      
	print "app stopped"
	expressionThread.stop()
	#likelihoodThread.stop()
	expressionThread.join()
	#likelihoodThread.join()
	print "thread stopped. exit."
