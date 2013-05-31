#!/usr/bin/env python
import roslib;
roslib.load_manifest('AccompanyService')
import rospy;
import sys
import os

import MySQLdb
#import mysql.connector
import json
import time
import math

from AccompanyService.srv import *

#class MyEncoder()
#class MySQLCursorDict(mysql.connector.cursor.MySQLCursor):
#class MySQLCursorDict(MySQLdb.connector.cursor.MySQLCursor)
#	def _row_to_python(self, rowdata, desc=None):
#		row = super(MySQLCursorDict, self)._row_to_python(row_data, desc)
#		if row:
#			return dict(zip(self.colum_names, row))
#		return None

HOST=""
LIKELIHOOD_TREESHOLD="0.1"
U=""
P=""
DATABASE_NAME=""

def handle_db_request(req):

	print "Received db request: %s [%s,%s]" % (req.request,req.param,req.sonReq)
	req_code=int(req.request)
	if req_code==1:
		print "full action list request..."
		db= MySQLdb.connect(HOST,U,P,DATABASE_NAME)
#		db= mysql.conector.connect(user="root",password="accompany",host="192.168.1.111",database="Accompany")
#		cur=db.cursor();
		cur=db.cursor()
#		sql= ('Select ap_label,command,phraseal_feedback,type_description,likelihood,apId,location_name,precond_id '
#                      'from (Select ActionPossibilities.text as ap_label,ActionPossibilities.command,ActionPossibilityType.text as type_description,parentId,ActionPossibilities.apId '
#                      ' as apId,ActionPossibilities.phraseal_feedback,likelihood,precond_id,Locations.name as location_name, Locations.locationId'
#                      ' from ActionPossibilities,ActionPossibilityType, Locations where ActionPossibilities.apTypeId=ActionPossibilityType.apTypeId and parentId is null and'
#                       ' Locations.locationId = ActionPossibilities.locationId and ActionPossibilities.likelihood>%s ) as pinco;')%LIKELIHOOD_TREESHOLD

		sql=('Select label_text,phrase,type_description,likelihood,apId,location_name,precondId from '
		     '(Select (Select message from Messages where messageId=a.ap_text and languageId=1) as label_text, '
		     'ActionPossibilityType.text as type_description,a.parentId,a.apId as apId,(Select message from Messages where messageId=a.ap_phrase and languageId=1) as phrase, '
		     'a.likelihood, a.precondId, Locations.name as location_name, Locations.locationId from ActionPossibilities a, ActionPossibilityType, Locations where '
		     'a.apTypeId = ActionPossibilityType.apTypeId and parentId is null and Locations.locationId = a.locationId and a.likelihood >%s ) as pinco;')%LIKELIHOOD_TREESHOLD
		cur.execute(sql)
		#print sql
		#a = cur.fetchall()
		#encoder= json.JSONEncoder()
		#print a
		#print "----"
		a="["
		#print "["
		for record in cur:
			#print "{"
			#print "\"ap_label\":\"%s\""%record[0]
			#print "}"
			a+="{"
			a+="\"ap_label\":\"%s\","%record[0]
	#		a+="\"command\":\"%s\","%record[1]
			a+="\"phraseal_feedback\":\"%s\","%record[1]
			a+="\"type_description\":\"%s\","%record[2]
			a+="\"likelihood\":\"%s\","%record[3]
			a+="\"apId\":\"%s\","%record[4]
			a+="\"location_name\":\"%s\","%record[5]
			a+="\"precond_id\":\"%s\""%record[6]
			a+="}"

		#print "]"
		a+="]"
		a=a.replace('}{','},{')
		#for record in cur:
			#a = a + encoder.encode(record)
		#	print encoder.encode(record)
		#print encoder.encode(cur.description)
		#print encoder.encode(cur.fetchone())
		#print encoder.encode(cur.fetchone())
	elif req_code==2:
		print "user actions request..."
		db= MySQLdb.connect(HOST,U,P,DATABASE_NAME)
		cur= db.cursor()
	 	#sql = ('Select ap_label,command,phraseal_feedback,type_description,likelihood,apId from '
		#      '(Select ActionPossibilities.text as ap_label,ActionPossibilities.command,ActionPossibilityType.text as type_description,parentId,ActionPossibilities.apId as '		
		#      'apId,ActionPossibilities.phraseal_feedback,likelihood,Conditions.conditionId as conditionId from ActionPossibilities,ActionPossibilityType ,Conditions, Locations '
		#      'where ActionPossibilities.apTypeId=ActionPossibilityType.apTypeId and ActionPossibilities.conditionId=Conditions.conditionId and parentId is null '
                #      'and Conditions.likelihood>%s and Locations.locationId=ActionPossibilities.locationId and Locations.locationId=\'%s\') as pinco order by likelihood desc;'%	(LIKELIHOOD_TREESHOLD,req.param))
#		sql = ('Select ap_label,command,phraseal_feedback,type_description,likelihood,apId,precond_id from '
#		      '(Select ActionPossibilities.text as ap_label,ActionPossibilities.command,ActionPossibilityType.text as type_description,parentId,ActionPossibilities.apId as '		
#		      'apId,ActionPossibilities.phraseal_feedback,likelihood,precond_id from ActionPossibilities,ActionPossibilityType, Locations '
#		      'where ActionPossibilities.apTypeId=ActionPossibilityType.apTypeId and parentId is null '
 #                     'and ActionPossibilities.likelihood>%s and Locations.locationId=ActionPossibilities.locationId and Locations.locationId=\'%s\') as pinco order by likelihood desc;'%	(LIKELIHOOD_TREESHOLD,req.param))
		sql=('Select label_text,phrase,type_description,likelihood,apId,precondId from '
		     '(Select (Select message from Messages where messageId=a.ap_text and languageId=1) as label_text, '
		     'ActionPossibilityType.text as type_description,a.parentId,a.apId as apId,(Select message from Messages where messageId=a.ap_phrase and languageId=1) as phrase, '
		     'a.likelihood, a.precondId from ActionPossibilities a, ActionPossibilityType, Locations where '
		     'a.apTypeId = ActionPossibilityType.apTypeId and parentId is null and Locations.locationId = a.locationId and Locations.locationId=\'%s\' and a.likelihood >%s ) as pinco order by likelihood desc;'%(req.param,LIKELIHOOD_TREESHOLD))
		#print sql
		#print req.param
		cur.execute(sql)
		a="["
		for record in cur:
			a+="{"
			a+="\"ap_label\":\"%s\","%record[0]
	#		a+="\"command\":\"%s\","%record[1]
			a+="\"phraseal_feedback\":\"%s\","%record[1]
			a+="\"type_description\":\"%s\","%record[2]
			a+="\"likelihood\":\"%s\","%record[3]
			a+="\"apId\":\"%s\","%record[4]
			a+="\"precond_id\":\"%s\""%record[5]
			a+="}"
		a+="]"	
		a=a.replace('}{','},{')
		#print a
	elif req_code==3:
		print "robot actions request..."
		db= MySQLdb.connect(HOST,U,P,DATABASE_NAME)
		cur= db.cursor()
#		sql = ('Select ap_label,command,phraseal_feedback,type_description,likelihood,apId,precond_id from '
#		      '(Select ActionPossibilities.text as ap_label,ActionPossibilities.command,ActionPossibilityType.text as type_description,parentId,ActionPossibilities.apId as '		
#		      'apId,ActionPossibilities.phraseal_feedback,likelihood,precond_id from ActionPossibilities,ActionPossibilityType, Locations '
#		      'where ActionPossibilities.apTypeId=ActionPossibilityType.apTypeId and parentId is null '
#                      'and ActionPossibilities.likelihood>%s and Locations.locationId=ActionPossibilities.locationId and Locations.locationId=\'%s\') as pinco order by likelihood desc;'%	(LIKELIHOOD_TREESHOLD,req.param))
		sql=('Select label_text,phrase,type_description,likelihood,apId,precondId from '
		     '(Select (Select message from Messages where messageId=a.ap_text and languageId=1) as label_text, '
		     'ActionPossibilityType.text as type_description,a.parentId,a.apId as apId,(Select message from Messages where messageId=a.ap_phrase and languageId=1) as phrase, '
		     'a.likelihood, a.precondId from ActionPossibilities a, ActionPossibilityType, Locations where '
		     'a.apTypeId = ActionPossibilityType.apTypeId and parentId is null and Locations.locationId = a.locationId and Locations.locationId=\'%s\' and a.likelihood >%s ) as pinco order by likelihood desc;'%(req.param,LIKELIHOOD_TREESHOLD))
		#print sql
		#print req.param
		cur.execute(sql)
		a="["
		for record in cur:
			a+="{"
			a+="\"ap_label\":\"%s\","%record[0]
#			a+="\"command\":\"%s\","%record[1]
			a+="\"phraseal_feedback\":\"%s\","%record[1]
			a+="\"type_description\":\"%s\","%record[2]
			a+="\"likelihood\":\"%s\","%record[3]
			a+="\"apId\":\"%s\","%record[4]
			a+="\"precond_id\":\"%s\""%record[5]
			a+="}"
		a+="]"	
		a=a.replace('}{','},{')
	elif req_code==4:
		print "requesting sons..."
		db= MySQLdb.connect(HOST,U,P,DATABASE_NAME)
		cur= db.cursor()
#		sql= ('Select  ap_label,command,phraseal_feedback,type_description,precond_id from (Select	ActionPossibilities.text as ap_label,command,ActionPossibilityType.text as '
#		      'type_description,parentId,ActionPossibilities.apId,phraseal_feedback,precond_id  from ActionPossibilities,ActionPossibilityType where '
#		      'ActionPossibilities.apTypeId=ActionPossibilityType.apTypeId and parentId=%s and '
#		      'ActionPossibilities.likelihood>%s ) as pinco;'%(req.param,LIKELIHOOD_TREESHOLD))

		sql= ('Select  label_text,phrase,type_description,precondId,likelihood from '
		     '(Select (Select message from Messages where messageId=a.ap_text and languageId=1) as label_text, '
		     'ActionPossibilityType.text as type_description,a.parentId,a.apId as apId,(Select message from Messages where messageId=a.ap_phrase and languageId=1) as phrase, '
		     'a.likelihood, a.precondId from ActionPossibilities a, ActionPossibilityType where '
		     'a.apTypeId = ActionPossibilityType.apTypeId and parentId=%s and a.likelihood >%s ) as pinco;'%(req.param,LIKELIHOOD_TREESHOLD))
		cur.execute(sql)
		#print sql
		a="["
		for record in cur:
			a+="{"
			a+="\"ap_label\":\"%s\","%record[0]
#			a+="\"command\":\"%s\","%record[1]
			a+="\"phraseal_feedback\":\"%s\","%record[1]
			a+="\"type_description\":\"%s\","%record[2]
			a+="\"precond_id\":\"%s\","%record[3]
			a+="\"likelihood\":\"%s\""%record[4]
			a+="}"
		a+="]"	
		a=a.replace('}{','},{')
	elif req_code==5:
		print "expression request..."
		db= MySQLdb.connect(HOST,U,P,DATABASE_NAME)
		cur= db.cursor()
		sql="Select expression from GUIexpression where ison=\'1\' limit 1;"
		cur.execute(sql)
		if cur.rowcount==0:
			a= "error"
		else:
			a="%s"%cur.fetchone()[0]
		print a
	elif req_code==0:
		print "login request..."
		print HOST
		print U
		print P
		print DATABASE_NAME
		db= MySQLdb.connect(HOST,U,P,DATABASE_NAME)
		cur= db.cursor()
		sql= ('Select userId from Users where nickname=\'%s\''%req.param)
		#sql = ('Select idAccompanyUser from AccompanyUser where nickname=\'%s\''%req.param)
		cur.execute(sql)
		if cur.rowcount==0: 
			a="-1"
		else:
			a="%s"%cur.fetchone()[0]
		print a
	elif req_code==10:
		print "local request for user position..."
		db= MySQLdb.connect(HOST,U,P,DATABASE_NAME)
		cur= db.cursor()
		##getting the object id associated to the user
		#sql = ('Select objectId from AccompanyUser where idAccompanyUser=\'%s\''%req.param)
		#cur.execute(sql)
		#if cur.rowcount==0: 
		#	a="-1"
		#else:
		#	sql = ('Select xCoord,yCoord,zCoord from Objects where objectId=\'%s\''%cur.fetchone()[0])
		#	cur.execute(sql)
		#	if cur.rowcount==0: 
		#		a="-1"
		#	else:	
		#		reslist=cur.fetchone()
		sql= 'Select  xCoord,yCoord,orientation from Locations where locationId=\'999\''
		cur.execute(sql)
		if cur.rowcount==0: 
			a="-1"
		else:	
			reslist=cur.fetchone()
			reslist2=[reslist[0],reslist[1],math.pi*reslist[2]/180];
			a= '['+ ','.join(str(n) for n in reslist2) +']'
		print a

	elif req_code==11:
		print "local request for object position..."
		db= MySQLdb.connect(HOST,U,P,DATABASE_NAME)
		cur= db.cursor()
		sql = ('Select xCoord,yCoord,zCoord from Objects where objectId=\'%s\''%cur.fetchone()[0])
		cur.execute(sql)
		if cur.rowcount==0: 
			a="-1"
		else:	
			a= '['+ ','.join(str(n) for n in reslist) +']'
		print a

	elif req_code==999:
		print "command received... (%s)"%(req.param)
		db= MySQLdb.connect(HOST,U,P,DATABASE_NAME)
		cur= db.cursor()
		sql = ('UPDATE Sensors SET value=\'1\' WHERE sensorId=\'%s\''%req.param)		
		cur.execute(sql);
		flag=1;
		time.sleep(0.25)
		while (flag==1):
			db= MySQLdb.connect(HOST,U,P,DATABASE_NAME)
			cur= db.cursor()
			sql=('SELECT value FROM Sensors WHERE sensorId=\'%s\''%req.param)
			cur.execute(sql);
			if cur.rowcount==0: 
				a="-1"
				flag=1;
				print 'Problem! rowcount=0'
			else:	
				reslist=cur.fetchone()
				flag=reslist[0]
				a=str(reslist[0])
				#print 'res:  %s'%a
			time.sleep(0.25)
		

	else:
		print "error on \'%s\'..."%req.request
		a="err"			

	return (req.request,req.sonReq,a)

def db_server():
	rospy.init_node('DatabaseConnectorService')
	s = rospy.Service('DatabaseConnectorService',db_msg,handle_db_request)
	print "Accompany Database service running..."
	rospy.spin()

if __name__=="__main__":
	#loading the coinfiguration
	#HOST="192.168.1.110"
	#LIKELIHOOD_TREESHOLD="0.1"
	#U="root"
	#P="accompany"
	#DATABASE_NAME="Accompany"
	path = roslib.packages.get_pkg_dir('AccompanyService')+"/src/accompany_service_cnf/db_cnf"
	print path
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

	db_server()

