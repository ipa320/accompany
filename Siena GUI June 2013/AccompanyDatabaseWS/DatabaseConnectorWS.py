#!/usr/bin/env python

import web
import sys
import os

# for database management
import MySQLdb
import json
import time
import math

urls = (
  '/', 'login',
  '/login', 'login',
  '/login/(.*)', 'login',
  '/full_action_list','full_action_list',
  '/full_action_list/(.*)','full_action_list',
  '/user_actions','user_actions',
  '/user_actions/(.*)','user_actions',
  '/robot_actions','robot_actions',
  '/robot_actions/(.*)','robot_actions',
  '/sons_actions','sons_actions',
  '/sons_actions/(.*)','sons_actions',
  '/expression_request','expression_request',
  '/command','command',
  '/command/(.*)','command',
  '/options','options',
  '/options/(.*)','options',
  '/setparameter','setparameter',
  '/setparameter/(.*)','setparameter'
)

class login:
	def GET(self):
		nick=web.input(unick='')
		nickname=web.websafe(nick.unick)
		#HOST="10.0.1.5"
		#LIKELIHOOD_TREESHOLD="0.1"
		#U="root"
		#P="accompany"
		#DATABASE_NAME="Accompany"

 		#read the db configuration from file
   	 	#path = "/home/patrizia/ros_workspace/AccompanyDatabaseWS/db_cnf"
   		#print path
		path = os.getcwd()+"/db_cnf"
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

		print "login request..."
		print "host: %s"%HOST
		print "user: %s"%U
		print "pwd:  %s"%P
		print "db:   %s"%DATABASE_NAME
		db= MySQLdb.connect(HOST,U,P,DATABASE_NAME)
		cur= db.cursor()
		sql= ('Select userId,languageId from Users where nickname=\'%s\''%nickname)
		cur.execute(sql)
		
		if cur.rowcount==0: 
			a="-1,-1"
		else:
			record=cur.fetchone()
			a="%s,%s"%(record[0],record[1])
		print a
		return a

class full_action_list:
	def GET(self):
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

		print "full action list request..."
		db= MySQLdb.connect(HOST,U,P,DATABASE_NAME)
		cur=db.cursor()
		sql=('Select label_text,phrase,type_description,likelihood,apId,location_name,precondId from '
		     '(Select (Select message from Messages where messageId=a.ap_text and languageId=1) as label_text, '
		     'ActionPossibilityType.text as type_description,a.parentId,a.apId as apId,(Select message from Messages where 		messageId=a.ap_phrase and languageId=1) as phrase, '
		     'a.likelihood, a.precondId, Locations.name as location_name, Locations.locationId from ActionPossibilities a, ActionPossibilityType, Locations where '
		     'a.apTypeId = ActionPossibilityType.apTypeId and parentId is null and Locations.locationId = a.locationId and a.likelihood >%s ) as pinco;')%LIKELIHOOD_TREESHOLD
		cur.execute(sql)
		a="["
		for record in cur:
			a+="{"
			a+="\"ap_label\":\"%s\","%record[0]
			a+="\"phraseal_feedback\":\"%s\","%record[1]
			a+="\"type_description\":\"%s\","%record[2]
			a+="\"likelihood\":\"%s\","%record[3]
			a+="\"apId\":\"%s\","%record[4]
			a+="\"location_name\":\"%s\","%record[5]
			a+="\"precond_id\":\"%s\""%record[6]
			a+="}"
		a+="]"
		a=a.replace('}{','},{')
		return a

class user_actions:
	def GET(self):
		user=web.input(uid='-1',ulang='-1')
		#user_language=web.input(ulang='-1')
		
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

		print "user actions request..."
		userId=web.websafe(user.uid)
		user_lang=web.websafe(user.ulang)
		db= MySQLdb.connect(HOST,U,P,DATABASE_NAME)
		cur= db.cursor()
	 	
		sql=('Select label_text,phrase,type_description,likelihood,apId,precondId from '
		     '(Select (Select message from Messages where messageId=a.ap_text and languageId=\'%s\') as label_text, '
		     'ActionPossibilityType.text as type_description,a.parentId,a.apId as apId,(Select message from Messages where messageId=a.ap_phrase and languageId=1) as phrase, '
		     'a.likelihood, a.precondId from ActionPossibilities a, ActionPossibilityType, Locations where '
		     'a.apTypeId = ActionPossibilityType.apTypeId and parentId is null and Locations.locationId = a.locationId and Locations.locationId=(Select locationId from Users where userId=\'%s\') and a.likelihood >%s ) as pinco order by likelihood desc;'%(user_lang,userId,LIKELIHOOD_TREESHOLD))
		cur.execute(sql)
		a="["
		for record in cur:
			a+="{"
			a+="\"ap_label\":\"%s\","%record[0]
			a+="\"phraseal_feedback\":\"%s\","%record[1]
			a+="\"type_description\":\"%s\","%record[2]
			a+="\"likelihood\":\"%s\","%record[3]
			a+="\"apId\":\"%s\","%record[4]
			a+="\"precond_id\":\"%s\""%record[5]
			a+="}"
		a+="]"	
		a=a.replace('}{','},{')
		return a

class robot_actions:
	def GET(self):
		paras=web.input(ulang='-1')
		lang=web.websafe(paras.ulang)
		
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

		print "robot actions request..."
		db= MySQLdb.connect(HOST,U,P,DATABASE_NAME)
		cur= db.cursor()
		sql=('Select label_text,phrase,type_description,likelihood,apId,precondId from '
		     '(Select (Select message from Messages where messageId=a.ap_text and languageId=1) as label_text, '
		     'ActionPossibilityType.text as type_description,a.parentId,a.apId as apId,(Select message from Messages where messageId=a.ap_phrase and languageId=\'%s\') as phrase, '
		     'a.likelihood, a.precondId from ActionPossibilities a, ActionPossibilityType, Locations where '
		     'a.apTypeId = ActionPossibilityType.apTypeId and parentId is null and Locations.locationId = a.locationId and Locations.locationId=(Select locationId from Robot where robotName=\'simulated\') and a.likelihood >%s ) as pinco order by likelihood desc;'%(lang,LIKELIHOOD_TREESHOLD))
		cur.execute(sql)
		a="["
		for record in cur:
			a+="{"
			a+="\"ap_label\":\"%s\","%record[0]
			a+="\"phraseal_feedback\":\"%s\","%record[1]
			a+="\"type_description\":\"%s\","%record[2]
			a+="\"likelihood\":\"%s\","%record[3]
			a+="\"apId\":\"%s\","%record[4]
			a+="\"precond_id\":\"%s\""%record[5]
			a+="}"
		a+="]"	
		a=a.replace('}{','},{')
		return a

class sons_actions:
	def GET(self):
		paras=web.input(ulang='-1',pid='-1')
		lang=web.websafe(paras.ulang)
		parent=web.websafe(paras.pid)
		
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

		print "requesting sons..."
		db= MySQLdb.connect(HOST,U,P,DATABASE_NAME)
		cur= db.cursor()
		sql= ('Select  label_text,phrase,type_description,precondId,likelihood,apId from '
		     '(Select (Select message from Messages where messageId=a.ap_text and languageId=\'%s\') as label_text, '
		     'ActionPossibilityType.text as type_description,a.parentId,a.apId as apId,(Select message from Messages where messageId=a.ap_phrase and languageId=1) as phrase, '
		     'a.likelihood, a.precondId from ActionPossibilities a, ActionPossibilityType where '
		     'a.apTypeId = ActionPossibilityType.apTypeId and parentId=%s and a.likelihood >%s ) as pinco;'%(lang,parent,LIKELIHOOD_TREESHOLD))
		cur.execute(sql)
		a="["
		for record in cur:
			a+="{"
			a+="\"ap_label\":\"%s\","%record[0]
			a+="\"phraseal_feedback\":\"%s\","%record[1]
			a+="\"type_description\":\"%s\","%record[2]
			a+="\"precond_id\":\"%s\","%record[3]
			a+="\"likelihood\":\"%s\","%record[4]
			a+="\"apId\":\"%s\""%record[5]
			a+="}"
		a+="]"	
		a=a.replace('}{','},{')
		return a

class expression_request:
	def GET(self):
		print "expression request..."
		
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
		sql="Select expression from GUIexpression where ison=\'1\' limit 1;"
		cur.execute(sql)
		if cur.rowcount==0:
			a= "error"
		else:
			a="%s"%cur.fetchone()[0]
		return a

class command:
	def GET(self):
		
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

		paras=web.input(cmd_id='-1')
		cmd=web.websafe(paras.cmd_id)
		print "command received... (%s)"%(cmd)
		db= MySQLdb.connect(HOST,U,P,DATABASE_NAME)
		cur= db.cursor()
		sql = ('UPDATE Sensors SET value=\'1\' WHERE sensorId=\'%s\''%cmd)		
		cur.execute(sql);
		return "ok"

class options:
	def GET(self):
		paras=web.input(ulang='-1',pid='-1')
		lang=web.websafe(paras.ulang)
		sonid=web.websafe(paras.pid)
		
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

		print "requesting options..."
		db= MySQLdb.connect(HOST,U,P,DATABASE_NAME)
		cur= db.cursor()
		sql= ('SELECT idActionPossibilityOptions as id, OptionName as name, PossibleValues as \'values\',DefaultValue FROM Accompany.ActionPossibilityOptions where idActionPossibilityOptions in (Select idOpt from ActionPossibility_APOptions where idAP=\'%s\');'%sonid)
		cur.execute(sql)
		a="["
		for record in cur:
			a+="{"
			a+="\"id\":\"%s\","%record[0]
			a+="\"name\":\"%s\","%record[1]
			a+="\"values\":\"%s\","%record[2]
			a+="\"default\":\"%s\""%record[3]
			a+="}"
		a+="]"	
		a=a.replace('}{','},{')
		return a

class setparameter:
	def GET(self):
		paras=web.input(opt_id='-1',val='-1')
		optId=web.websafe(paras.opt_id)
		value=web.websafe(paras.val)
		
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

		print "set parameter..."
		db= MySQLdb.connect(HOST,U,P,DATABASE_NAME)
		cur= db.cursor()
		sql = ('UPDATE Accompany.ActionPossibilityOptions SET SelectedValue=\'%s\' WHERE idActionPossibilityOptions=\'%s\''%(value,optId))		
		cur.execute(sql);
		return "ok"

if __name__ == "__main__": 
    
    #read the db configuration from file
    #path = "/home/patrizia/ros_workspace/AccompanyDatabaseWS/db_cnf"
    #print path
    ospath= os.getcwd()+"/db_cnf"
    print "path: %s"%ospath
    f= open(ospath)
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

    print "likelihood limit: %s"%LIKELIHOOD_TREESHOLD
    print "host:             %s"%HOST
    print "user:             %s"%U
    print "pwd:              %s"%P
    print "db:               %s"%DATABASE_NAME
    print "***************************"
    app = web.application(urls, globals())
    app.run()        
