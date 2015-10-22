import MySQLdb
import sys
import math
import time
import datetime
"""Handles all access to the database"""
"""Everything eventually filters into either SQLDao.getData or SQLDao.saveData"""
"""Pattern for all select statements: """
""" Table names are resolved using python string comprehension"""
""" Args are passed as a dictionary into the sql library which handles escaping and inserting into the query"""
""" This can get a bit confusing because they both look like python string comprehension, however when"""
"""   attempting to pass table names as args into sql, it will fail so we're stuck doing it this way"""
"""Note: dataAccess.DataAccess is only kept for legacy support, going forward functions are categorised into their """
"""    own classes.  Additionally, while table names can be passed into some of the constructors, this was dropped in """
"""    later classes in favour of always reading from the config object"""


class Locations(object):

    def __init__(self, robotTable=None, userTable=None, locationTable=None, locationRange=None):
        from config import server_config
        self._robotTable = robotTable or server_config['mysql_robot_table']
        self._userTable = userTable or server_config['mysql_users_table']
        self._locationTable = locationTable or server_config['mysql_location_table']
        self._experimentLocationTable = server_config['mysql_experimentLocations_table']
        self._sessionControlTable = server_config['mysql_session_control_table']
        self._sql = SQLDao()

        if locationRange is None:
            activeLocation = self.getActiveExperimentLocation()
            if activeLocation is None:
                locationRange = [0, 0]
            else:
                locationRange = (activeLocation['startLocationRange'], activeLocation['endLocationRange'])

        self._sqlQuery = 'SELECT * FROM `%s`' % (self._locationTable)
        self._locationFilter = " `locationId` >= %(min)s AND `locationId` <= %(max)s" % {
            'min': locationRange[0],
            'max': locationRange[1]
        }

    def getActiveExperimentLocation(self):
        sql = "SELECT \
                   `%(experimentLocations)s`.* \
               FROM \
                  `%(experimentLocations)s` INNER JOIN `%(session)s` ON \
                  `%(experimentLocations)s`.`id` = `%(session)s`.`ExperimentalLocationId`" % {
            'experimentLocations': self._experimentLocationTable,
            'session': self._sessionControlTable
        }
        data = self._sql.getSingle(sql)

        return data

    def getLocation(self, locationId):
        sql = self._sqlQuery
        sql += " WHERE `locationId` = %(locid)s"
        args = {'locid': locationId}

        return self._sql.getSingle(sql, args)

    @staticmethod
    def resolveLocation(curPos, maxDistance=None, dao=None):
        if(dao is None):
            dao = Locations()

        try:
            locations = dao.findLocations()
        except Exception:
            return ('', curPos)

        if len(locations) == 0:
            return ('', curPos)

        name = None
        diff = None
        for loc in locations:
            dist = 0
            dist += math.pow(curPos[0] - loc['xCoord'], 2)
            dist += math.pow(curPos[1] - loc['yCoord'], 2)
            # We're not worried about orientation for location matching
            # use a fraction of the orientation in case we get two named
            # locations with the same x/y and different orientations
            # needs to be a very small fraction since rotation uses a different unit measurement
            # than location does (degrees vs. meters)
            dist += math.pow(curPos[2] - loc['orientation'], 2) / 100000
            dist = math.sqrt(dist)
            if name is None or dist < diff:
                name = loc['name']
                diff = dist

        if maxDistance is None or diff <= maxDistance:
            return (name, curPos)
        else:
            return ('', curPos)

    def getLocationByName(self, name):
        sql = self._sqlQuery
        sql += " WHERE `name` = %(name)s"
        args = {'name': name}

        sql += " AND " + self._locationFilter

        return self._sql.getSingle(sql, args)

    def findLocations(self, locationName=None):
        sql = self._sqlQuery
        args = None
        if locationName is not None:
            sql += " WHERE `locationId` like %(locid)s"
            args = {'locid': locationName}
            sql += " AND " + self._locationFilter
        else:
            sql += " WHERE " + self._locationFilter

        return self._sql.getData(sql, args)

    def saveLocation(self, pk, locid, x, y, orientation, table, pkField):
        sql = 'UPDATE `%s` ' % (table)
        sql += 'SET `xCoord` = %(x)s, \
                    `yCoord` = %(y)s, \
                    `locationId` = %(locid)s, \
                    `orientation` = %(orien)s \
                WHERE `' + pkField + '` = %(pk)s'
        args = {
            'pk': pk,
            'locid': locid,
            'x': x,
            'y': y,
            'orien': orientation
        }

        return self._sql.saveData(sql, args) >= 0

    def saveRobotLocation(self, robotid, locid, x, y, orientation):
        return self.saveLocation(robotid, locid, x, y, orientation, self._robotTable, 'robotId')

    def saveUserLocation(self, userId, locid, x, y, orientation):
        return self.saveLocation(userId, locid, x, y, orientation, self._userTable, 'userId')


class UserInterface(object):

    def __init__(self, questionTable=None, responseTable=None):
        from config import server_config
        self._guiQuestionTable = questionTable or server_config['mysql_questions_table']
        self._responseTable = responseTable or server_config['mysql_responses_table']
        self._sql = SQLDao()

    def getActiveQuestion(self):
        sql = 'SELECT * FROM `%s`' % (self._guiQuestionTable)
        sql += ' WHERE `guiMsgResult`is NULL'
        sql += ' ORDER BY `timestamp` desc'

        data = self._sql.getSingle(sql)
        if data is not None:
            return {
                'sequenceName': data['name'],
                'response': data['guiMsgResult'],
                'timestamp': data['timestamp']
            }
        else:
            return None

    def setResponse(self, sequenceName, responseId):
        sql = 'UPDATE `%s` ' % (self._guiQuestionTable)
        sql += 'SET `guiMsgResult` = %(resp)s WHERE `name` = %(seqName)s'
        args = {
            'resp': responseId,
            'seqName': sequenceName
        }

        return self._sql.saveData(sql, args) >= 0

    def getResponses(self, sequenceName=None):
        sql = 'SELECT * FROM `%s`' % (self._responseTable)
        args = None
        if sequenceName is not None:
            sql += ' WHERE `name` = %(seqName)s'
            args = {'seqName': sequenceName}

        data = []
        sizer = lambda x: 'l' if int(x) > 0 else 's'
        for response in self._sql.getData(sql, args):
            for msgId in range(1, 5):
                if len(response['guiMsg%s' % (msgId)].strip()) > 0:
                    data.append({
                                 'guiResponseId':msgId,
                                 'message':response['guiMsg%s' % (msgId)],
                                 'size': sizer(response['guiMsg%sEnlarged' % (msgId)]),
                                 'seqName':response['name']
                                 })
        return data


class Users(object):

    def __init__(self, userTable=None, locationTable=None, userPreferencesTable=None, personasTable=None):
        from config import server_config
        self._userTable = userTable or server_config['mysql_users_table']
        self._locationTable = locationTable or server_config['mysql_location_table']
        self._sessionControlTable = server_config['mysql_session_control_table']
        self._userPreferencesTable = userPreferencesTable or server_config['mysql_user_preferences_table']
        self._personasTable = personasTable or server_config['mysql_personas_table']
        self._sql = SQLDao()

    def getUser(self, userId):
        sql = "SELECT `%(user)s`.*, %(loc)s.name as 'locationName' \
               FROM `%(user)s` \
               INNER JOIN `%(loc)s` ON %(loc)s.`locationId` = `%(user)s`.`locationId`" % {
            'user': self._userTable,
            'loc': self._locationTable
        }
        sql += " WHERE `userId` = %(uid)s"
        args = {'uid': userId}

        return self._sql.getSingle(sql, args)

    def getUserByNickName(self, userName):
        sql = "SELECT `%(user)s`.*, %(loc)s.name as 'locationName' \
               FROM `%(user)s` \
               INNER JOIN `%(loc)s` ON %(loc)s.`locationId` = `%(user)s`.`locationId`" % {
            'user': self._userTable,
            'loc': self._locationTable
        }
        sql += " WHERE `nickname` = %(name)s"
        args = {'name': userName}

        return self._sql.getSingle(sql, args)

    def getUserByName(self, userName):
        sql = "SELECT `%(user)s`.*, %(loc)s.name as 'locationName' \
               FROM `%(user)s` \
               INNER JOIN `%(loc)s` ON %(loc)s.`locationId` = `%(user)s`.`locationId`" % {
            'user': self._userTable,
            'loc': self._locationTable
        }
        sql += " WHERE `firstName` = %(name)s"
        args = {'name': userName}

        return self._sql.getSingle(sql, args)

    def findUsers(self, userName=None):
        sql = "SELECT `%(user)s`.*, %(loc)s.name as 'locationName' \
               FROM `%(user)s` \
               INNER JOIN `%(loc)s` ON %(loc)s.`locationId` = `%(user)s`.`locationId`" % {
            'user': self._userTable,
            'loc': self._locationTable
        }
        args = None
        if userName is not None:
            sql += " WHERE `firstName` like %(name)s"
            args = {'name': userName}

        return self._sql.getData(sql, args)

    def updateUser(self, user):
        sql = "UPDATE `%(user)s` " % {'user': self._userTable}
        sql += "\
                SET \
                    `firstName` = %(firstName)s, \
                    `lastName` = %(lastName)s, \
                    `nickname` = %(nickname)s, \
                    `locationId` = %(locationId)s, \
                    `poseId` = %(poseId)s, \
                    `contextId` = %(contextId)s, \
                    `uniqueRobotHouseUser` = %(uniqueRobotHouseUser)s, \
                    `languageId` = %(languageId)s, \
                    `xCoord` = %(xCoord)s, \
                    `yCoord` = %(yCoord)s, \
                    `orientation` = %(orientation)s \
                WHERE \
                    `userId` = %(userId)s"
        if self._sql.saveData(sql, user) >= 0:
            return True
        else:
            return False

    def setActiveUser(self, uid, sid=1):
        sql = 'UPDATE `%s` ' % (self._sessionControlTable)
        sql += ' \
                SET \
                    `sessionTime` = %(time)s, \
                    `sessionDate` = %(date)s, \
                    `SessionUser` = %(user)s \
                WHERE \
                    `sessionId` = %(sid)s'

        args = {'time': time.strftime('%X'), 'date': time.strftime('%Y-%m-%d'), 'user': uid, 'sid': sid}

        if self._sql.saveData(sql, args) >= 0:
            return sid
        else:
            return None

    def getActiveUser(self):
        sql = "SELECT `%(user)s`.*, %(loc)s.name as 'locationName' \
               FROM `%(user)s` \
               INNER JOIN `%(loc)s` ON %(loc)s.`locationId` = `%(user)s`.`locationId` \
               INNER JOIN `%(session)s` s ON s.`SessionUser` = `%(user)s`.`userId`" % {
            'user': self._userTable,
            'loc': self._locationTable,
            'session': self._sessionControlTable
        }
        sql += " WHERE s.`sessionId` = %(sid)s"
        args = {'sid': 1}

        return self._sql.getSingle(sql, args)

    def getUserPreferences(self):
        sql = "SELECT `%(user)s`.* \
               FROM `%(user)s` \
               WHERE `%(user)s`.`userId` IN ( \
                        SELECT `%(session)s`.`sessionUser` \
                        FROM `%(session)s` )" % {
            'user': self._userPreferencesTable,
            'session': self._sessionControlTable
        }
        args = None

        return self._sql.getData(sql, args)

    def setUserPreferences(self, column, value):
        sql = "UPDATE `%(preferences)s` \
               SET `%(column)s` = %(value)s \
               WHERE `%(preferences)s`.`userId` IN ( \
                        SELECT `%(session)s`.`sessionUser` \
                        FROM `%(session)s` )" % {
            'preferences': self._userPreferencesTable,
            'session': self._sessionControlTable,
            'column': column,
            'value': "%(value)s"
        }
        args = {
            'value': value
        }

        return self._sql.saveData(sql, args) >= 0

    def getPersonaValues(self):
        sql = "SELECT `%(persona)s`.* \
               FROM `%(persona)s` \
               WHERE `%(persona)s`.`personaId` IN ( \
                        SELECT `%(user)s`.`personaId` \
                        FROM `%(user)s`, `%(session)s` \
                        WHERE `%(session)s`.`sessionUser` = `%(user)s`.`userId` )" % {
            'persona': self._personasTable,
            'user': self._userTable,
            'session': self._sessionControlTable
        }
        args = None

        return self._sql.getData(sql, args)


class Robots(object):

    def __init__(self, robotTable=None, locationTable=None):
        from config import server_config
        self._robotTable = robotTable or server_config['mysql_robot_table']
        self._locationTable = locationTable or server_config['mysql_location_table']
        self._sessionControlTable = server_config['mysql_session_control_table']
        self._sql = SQLDao()

    def getRobot(self, robotId):
        sql = "SELECT `%(rob)s`.*, %(loc)s.name as 'locationName' \
               FROM `%(rob)s` \
               INNER JOIN `%(loc)s` ON %(loc)s.`locationId` = `%(rob)s`.`locationId`" % {
            'rob': self._robotTable,
            'loc': self._locationTable
        }
        sql += " WHERE `robotid` = %(name)s"
        args = {'name': robotId}

        return self._sql.getSingle(sql, args)

    def getRobotByName(self, robotName):
        sql = "SELECT `%(rob)s`.*, %(loc)s.name as 'locationName' \
               FROM `%(rob)s` \
               INNER JOIN `%(loc)s` ON %(loc)s.`locationId` = `%(rob)s`.`locationId`" % {
            'rob': self._robotTable,
            'loc': self._locationTable
        }
        sql += " WHERE `robotName` = %(name)s"
        args = {'name': robotName}

        return self._sql.getSingle(sql, args)

    def findRobots(self, robotName=None):
        sql = "SELECT `%(rob)s`.*, %(loc)s.name as 'locationName' \
               FROM `%(rob)s` \
               INNER JOIN `%(loc)s` ON %(loc)s.`locationId` = `%(rob)s`.`locationId`" % {
            'rob': self._robotTable,
            'loc': self._locationTable
        }
        args = None
        if robotName is not None:
            sql += " WHERE `robotName` like %(name)s"
            args = {'name': robotName}

        return self._sql.getData(sql, args)

    def getActivatedRobot(self):
        sql = "SELECT `%(rob)s`.`robotName` \
               FROM `%(rob)s` \
               INNER JOIN `%(session)s` ON `%(session)s`.`robotId` = `%(rob)s`.`robotId`" % {
            'rob': self._robotTable,
            'session': self._sessionControlTable
        }

        return self._sql.getData(sql)


class ActionHistory(object):

    def __init__(self, actionHistoryTable=None, sensorSnapshotTable=None, sensorTable=None, sensorTypeTable=None, locationTable=None):
        from config import server_config
        self._historyTable = actionHistoryTable or server_config['mysql_history_table']
        self._sensorHistoryTable = sensorSnapshotTable or server_config['mysql_sensorHistory_table']
        self._sensorTable = sensorTable or server_config['mysql_sensor_table']
        self._locationTable = locationTable or server_config['mysql_location_table']
        self._sensorTypeTable = sensorTypeTable or server_config['mysql_sensorType_table']
        self._userTable = userTable or server_config['mysql_users_table']
        self._scenarioTable = scenarioTable or server_config['mysql_scenario_table']
        self._sql = SQLDao()
        self._selectQuery = "SELECT `actionHistoryId`, `imageId`, `imageOverheadId`, `timestamp`, `ruleName`, `tags`,\
               `%(loc)s`.`name` as 'location', `%(user)s`.`nickname` as 'nickname', `%(scen)s`.`name` as 'scenario', \
               `%(scen)s`.`description` as 'description', `%(scen)s`.`image` as 'imageArt',\
               `%(scen)s`.`shortDescription` as 'shortDescription'\
               FROM `%(hist)s` \
               INNER JOIN `%(user)s` ON `%(user)s`.`userId` = `%(hist)s`.`userId`\
               INNER JOIN `%(scen)s` ON `%(scen)s`.`scenarioId` = `%(hist)s`.`scenarioId`\
               INNER JOIN `%(loc)s` ON `%(loc)s`.`locationId` = `%(hist)s`.`locationId`" % {
                                                                                            'hist' : self._historyTable,
                                                                                            'loc': self._locationTable,
                                                                                            'user': self._userTable,
                                                                                            'scen': self._scenarioTable
                                                                                           }
    def getHistoryOrdered(self, ruleName=None, tags=None):
        sql = self._selectQuery
        args = {}
        where = ''
        if ruleName != None:
            where = self._appendWhere(where, "`ruleName` = %(name)s")
            args['name'] = ruleName
        if tags != None and len(tags) > 0:
            clause = ''
            for index in range(0, len(tags)):
                tag = tags[index]
                key = 'tag_' + str(index)
                clause += "`tags` like %(" + key + ")s"
                if index + 1 != len(tags):
                    clause += " OR "
                args[key] = "%'" + tag + "'%"
            clause = '(' + clause + ')'
            where = self._appendWhere(where, clause)

        sql += where
        sql += ' ORDER BY `timestamp` asc'
        return self._processResultsOrdered(self._sql.getData(sql, args))

    def getHistory(self, ruleName=None, tags=None):
        sql = self._selectQuery
        args = {}
        where = ''
        if ruleName is not None:
            where = self._appendWhere(where, "`ruleName` = %(name)s")
            args['name'] = ruleName
        if tags is not None and len(tags) > 0:
            clause = ''
            for index in range(0, len(tags)):
                tag = tags[index]
                key = 'tag_' + str(index)
                clause += "`tags` like %(" + key + ")s"
                if index + 1 != len(tags):
                    clause += " OR "
                args[key] = "%'" + tag + "'%"
            clause = '(' + clause + ')'
            where = self._appendWhere(where, clause)

        sql += where
        return self._processResults(self._sql.getData(sql, args))

    def _appendWhere(self, where, clause):
        if where is None or where == '' or where.strip() == '':
            where = " WHERE "
        else:
            where = " AND "
        where += clause
        return where

    def _processResults(self, data):
        result = []
        for row in data:
            obj = {
                       'id': row['actionHistoryId'],
                       'name':row['ruleName'],
                       'status': 'activate',
                       'imageId':row['imageId'],
                       'imageOverheadId':row['imageOverheadId'],
                       'location': row['location'],
                       'tags': eval(row['tags'] or '()'),
                       'time': {
                                'real': row['timestamp'].strftime('%Y-%m-%d %H:%M:%S'),
                                'narrative': row['timestamp'].strftime('%Y%m%d%H%M%S')
                                },
                        #'sensors': self.getSensorSnapshot(row['actionHistoryId'])
                       }
            
            if type(obj['tags']) != list and type(obj['tags']) != tuple:
                obj['tags'] = (obj['tags'],)
            result.append(obj)
            
        return result

    def _processResultsOrdered(self, data):
        result = []
        events = []
        for row in data:
            #if not row['scenario'] in events:
            #  events.append(row['scenario'])
            if row['ruleName'] == 'end':#added 'end' at the end of each episode in DB
              events.append(row['scenario'])

        _eventid=0
        for e in events:
          elements = {
                        'eventId':str(_eventid),
                        'scenario':e,
                        'description':[],
                        'shortDescription':[],
                        'endTime':[],
                        'imageArt':[],
                        'whentime':[],
                        'events':[]
                        }
          _eventid=_eventid+1
          for row in data:
              if e == row['scenario']:
                obj = {
                           'historyId': row['actionHistoryId'],
                           'name':row['ruleName'],
                           'status': 'activate',
                           'imageId':row['imageId'],
                           'imageOverheadId':row['imageOverheadId'],
                           'location': row['location'],
                           'nickname':row['nickname'],
                           'scenario':row['scenario'],
                           'tags': eval(row['tags'] or '()'),
                           'time': {
                                    'real': row['timestamp'].strftime('%Y-%m-%d %H:%M:%S'),
                                    'narrative': row['timestamp'].strftime('%Y%m%d%H%M%S')
                                    }
                           #'sensors': self.getSensorSnapshot(row['actionHistoryId'])
                           }

                if type(obj['tags']) != list and type(obj['tags']) != tuple:
                    obj['tags'] = (obj['tags'],)
              
                elements['events'].append(obj)
                elements['description']=row['description']
                elements['imageArt']=row['imageArt']
                elements['shortDescription']=row['shortDescription']
                elements['endTime']=row['timestamp'].strftime('%Y%m%d%H%M%S')
                elements['whentime']=self._processDate(row['timestamp'])

          result.append(elements)
        return result

    def _processDate(self,data):
        
        from datetime import datetime
        FMT='%Y-%m-%d %H:%M:%S'
        now=datetime.now()

        date=data.strftime('%Y-%m-%d %H:%M:%S')
        totaldelta=datetime.strptime(now.strftime(FMT),FMT)-datetime.strptime(date,FMT)

        midnight=now.strftime('%Y-%m-%d')+' 00:00:00'
        deltatm=datetime.strptime(now.strftime(FMT),FMT)-datetime.strptime(midnight,FMT)

        if totaldelta<deltatm:
          return data.strftime('%H:%M:%S')
        else:
          sym="%02d" % (int(now.strftime('%d'))-1) #str(int(now.strftime('%d'))-1)
          yesterdaymidnight=now.strftime('%Y-%m')+ '-' + sym + ' 00:00:00'
          deltaytm=datetime.strptime(now.strftime(FMT),FMT)-datetime.strptime(yesterdaymidnight,FMT)
          if totaldelta < deltaytm:
            return 'Yesterday ' + data.strftime('%H:%M:%S')
          else:
            sym=int(now.strftime('%d'))-6
            if sym>0:
              sym="%02d" % sym
            else:
              sym= "%02d" % (31+sym)
            lastweekmidnight=now.strftime('%Y-%m')+ '-' + sym + ' 00:00:00'
            #deltalwtm=datetime.strptime(now.strftime(FMT),FMT)-datetime.strptime(lastweekmidnight,FMT)
            #if totaldelta < deltalwtm:
            return data.strftime('%A') + ' '+ data.strftime('%H:%M:%S')
            #else:
            #  return str(totaldelta.days) +' days ago. (' + data.strftime('%Y-%m-%d') + ')'

    def getSensorSnapshot(self, historyId):
        sql = "SELECT `%(sensor)s`.`name` as `name`, `%(sensor)s`.`sensorId` as `sensorId`, `%(sensor)s`.`sensorRule` as `sensorRule`, \
                `%(loc)s`.`name` as `location`, `%(hist)s`.`sensorValue` as `value`, `%(type)s`.`sensorType` as `sensorTypeName`\
        FROM `%(hist)s` \
        INNER JOIN `%(sensor)s` ON `%(sensor)s`.`sensorId` = `%(hist)s`.`sensorId`\
        INNER JOIN `%(type)s` ON `%(sensor)s`.`sensorTypeId` = `%(type)s`.`sensorTypeId`\
        INNER JOIN `%(loc)s` ON `%(loc)s`.`locationId` = `%(sensor)s`.`locationId` " % {
            'hist': self._sensorHistoryTable,
            'loc': self._locationTable,
            'sensor': self._sensorTable,
            'type': self._sensorTypeTable
        }
        sql += " WHERE `actionHistoryId` = %(histid)s"
        args = {'histid': historyId}

        return self._sql.getData(sql, args)

    def saveSensorSnapshot(self, historyId):
        rowCount = 0
        for row in Sensors().findSensors():
            sensorId = row['sensorId']
            sensorValue = row['value']

            sql = "INSERT INTO `%s` (`actionHistoryId`, `sensorId`, `sensorValue`)" % (self._sensorHistoryTable)
            sql += " VALUES (%(histid)s, %(sensorid)s, %(value)s)"
            args = {
                'histid': historyId,
                'sensorid': sensorId,
                'value': sensorValue
            }

            if self._sql.saveData(sql, args) >= 0:
                rowCount += 1

        return rowCount
    
    def saveHistory(self, timestamp, ruleName, locationId):
        sql = "INSERT INTO `%s` (`timestamp`, `ruleName`, `locationId`)" % (self._historyTable) 
        sql += " VALUES (%(time)s, %(rule)s, %(locid)s)"
        args = {
            'time': timestamp,
            'rule': ruleName,
            'locid': locationId
        }

    def saveHistoryComplete(self, timestamp, ruleName, locationId, userId):
        sql = "INSERT INTO `%s` (`timestamp`, `ruleName`, `locationId`,`userId`)" % (self._historyTable) 
        sql += " VALUES (%(time)s, %(rule)s, %(locid)s, %(usr)s)" 
        args = {
                  'time' : timestamp,
                  'rule': ruleName,
                  'locid':locationId,
                  'usrid':userId
                  }
        
        return self._sql.saveData(sql, args)

    def updateTags(self, historyId, tags):
        sql = "UPDATE `%s`" % (self._historyTable)
        sql += " SET `timestamp` = `timestamp`,`tags` = %(tags)s WHERE `actionHistoryId` = %(histid)s"
        args = {
            'tags': str(tags),
            'histid': historyId
        }

        self._sql.saveData(sql, args)
        return historyId

    def saveHistoryImage(self, historyId, imageBytes, imageType):
        imageId = Binary().saveBinary('Image for history %s' % (historyId), imageBytes, imageType)
        sql = "UPDATE `%s`" % (self._historyTable)
        sql += " SET `imageId` = %(imgid)s WHERE `actionHistoryId` = %(histid)s"
        args = {
            'imgid': imageId,
            'histid': historyId
        }

        self._sql.saveData(sql, args)
        return imageId

    def saveHistoryImageOverhead(self, historyId, imageBytes, imageType):
        imageOverheadId = Binary().saveBinaryOverhead('ImageOverhead for history %s' % (historyId), imageBytes, imageType)
        sql = "UPDATE `%s`" % (self._historyTable)
        sql += " SET `imageOverheadId` = %(imgoid)s WHERE `actionHistoryId` = %(histid)s"
        args = {
            'imgoid': imageOverheadId,
            'histid': historyId
        }

        self._sql.saveData(sql, args)
        return imageOverheadId


class Sensors(object):

    def __init__(self, sensorTable=None, sensorTypeTable=None, sensorLogTable=None, locationTable=None, sensorRange=None, sensorIconTable=None):
        from config import server_config
        self._sensorTable = sensorTable or server_config['mysql_sensor_table']
        self._sensorTypeTable = sensorTypeTable or server_config['mysql_sensorType_table']
        self._sensorIconTable = sensorIconTable or server_config['mysql_sensorIcon_table']
        self._sensorLogTable = sensorLogTable or server_config['mysql_log_table']
        self._locationTable = locationTable or server_config['mysql_location_table']

        self._sql = SQLDao()

        if sensorRange is None:
            activeLocation = Locations().getActiveExperimentLocation()
            if activeLocation is None:
                sensorRange = [0, 0]
            else:
                sensorRange = (activeLocation['startSensorRange'], activeLocation['endSensorRange'])

        self._sqlQuery = "SELECT `%(sensors)s`.*, \
                                 `%(type)s`.`sensorType` as `sensorTypeName`, \
                                 `%(loc)s`.`name` as `locationName`, \
                                 `%(type)s`.`active` = `%(sensors)s`.`status` as `isActive` \
                FROM `%(sensors)s` \
                INNER JOIN `%(type)s` ON `%(sensors)s`.`sensorTypeId` = `%(type)s`.`sensorTypeId`\
                INNER JOIN `%(loc)s` ON `%(loc)s`.`locationId` = `%(sensors)s`.`locationId`" % {
            'sensors': self._sensorTable,
            'type': self._sensorTypeTable,
            'loc': self._locationTable}

        self._sqlQuery += " WHERE ((`sensorId` >= %(min)s AND `sensorId` <= %(max)s) OR (`sensorId` >= 500 AND `sensorId` <= 799 )) " % {
            'min': sensorRange[0],
            'max': sensorRange[1]
        }

    def getSensor(self, sensorId):
        sql = self._sqlQuery
        sql += " AND `sensorId` = %(sid)s"
        args = {'sid': sensorId}

        return self._sql.getSingle(sql, args)

    def getSensorByName(self, sensorName):
        sql = self._sqlQuery
        sql += " AND `name` = %(name)s"
        args = {'name': sensorName}

        return self._sql.getSingle(sql, args)

    def getSensorType(self, sensorTypeId):
        sql = "SELECT * FROM `%s` " % self._sensorTypeTable
        sql += " WHERE `sensorTypeId` = %(sid)s"
        args = {'sid': sensorTypeId}

        return self._sql.getSingle(sql, args)

    def getSensorTypeByName(self, sensorTypeName):
        sql = "SELECT * FROM `%s` " % self._sensorTypeTable
        sql += " WHERE `sensorType` = %(name)s"
        args = {'name': sensorTypeName}

        return self._sql.getSingle(sql, args)

    def getSensorIcon(self, sensorTypeId):
        sql = "SELECT * FROM `%s` " % self._sensorIconTable
        sql += " WHERE `id` = %(sid)s"
        args = {'sid': sensorTypeId}

        return self._sql.getSingle(sql, args)

    def getSensorIconByName(self, sensorTypeName):
        sql = "SELECT * FROM `%s` " % self._sensorIconTable
        sql += " WHERE `name` = %(name)s"
        args = {'name': sensorTypeName}

        return self._sql.getSingle(sql, args)

    def findSensors(self, sensorName=None, onlyPhysical=True):
        args = None
        sql = self._sqlQuery
        if sensorName is not None:
            sql += " AND `%s`.`name` like" % (self._sensorTable) + " %(name)s"
            args = {'name': sensorName}
        if onlyPhysical:
            sql += " AND `%s`.`ChannelDescriptor` != 'N/A' AND `%s`.`ChannelDescriptor` != ''" % (self._sensorTable, self._sensorTable)

        return self._sql.getData(sql, args)

    def saveSensorLog(self, sensorId, value, status, timestamp=None, room='', channel=''):
        timestamp = timestamp or datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')

        sql = "INSERT INTO `%(log)s`" % {'log': self._sensorLogTable}
        sql += "(`timestamp`, `sensorId`, `room`, `channel`, `value`, `status`) \
                    VALUES (%s, %s, %s, %s, %s, %s)"

        args = (timestamp,
                sensorId,
                room,
                channel,
                str(value),
                status)

        if self._sql.saveData(sql, args) >= 0:
            return True
        else:
            return False


class Binary(object):

    def __init__(self, binaryTable=None, binaryIdCol=None):
        from config import server_config
        self._binaryTable = binaryTable or server_config['mysql_image_table']
        self._binaryTableOverhead = server_config['mysql_imageoverhead_table']
        self._binaryIdCol = binaryIdCol or 'imageId'
        self._binaryOverheadIdCol = 'imageOverheadId'
        self._sql = SQLDao()

    def getBinary(self, itemId):
        sql = 'SELECT * FROM `%(table)s` WHERE `%(idCol)s`' % {'table': self._binaryTable, 'idCol': self._binaryIdCol}
        sql += " = %(id)s"
        args = {'id': itemId}

        data = self._sql.getSingle(sql, args)
        if data is not None:
            return {
                'data': data['bytes'],
                'meta': {
                    'type': data['type'],
                    'name': data['name']
                }}
        else:
            return {
                'data': None,
                'meta': {
                    'type': '',
                    'name': ''
                }
            }

    def getBinaryOverhead(self, itemId):
        sql = 'SELECT * FROM `%(table)s` WHERE `%(idCol)s`' % {'table': self._binaryTableOverhead, 'idCol': self._binaryOverheadIdCol}
        sql += " = %(id)s"
        args = {'id': itemId}

        data = self._sql.getSingle(sql, args)
        if data is not None:
            return {
                'data': data['bytes'],
                'meta': {
                    'type': data['type'],
                    'name': data['name']
                }}
        else:
            return {
                'data': None,
                'meta': {
                    'type': '',
                    'name': ''
                }
            }

    def saveBinary(self, itemName, itemBytes, itemType):
        sql = "INSERT INTO `%s` (`type`, `name`, `bytes`)" % (self._binaryTable)

        sql += " VALUES (%(type)s, %(name)s, %(bytes)s)"
        args = {
            'type': itemType,
            'name': itemName,
            'bytes': itemBytes
        }

        return self._sql.saveData(sql, args)

    def saveBinaryOverhead(self, itemName, itemBytes, itemType):
        sql = "INSERT INTO `%s` (`type`, `name`, `bytes`)" % (self._binaryTableOverhead)

        sql += " VALUES (%(type)s, %(name)s, %(bytes)s)"
        args = {
            'type': itemType,
            'name': itemName,
            'bytes': itemBytes
        }

        return self._sql.saveData(sql, args)


class DataAccess(object):
    """Obsolete, use individual classes"""

    def __init__(self, config=None):
        self._locations = None
        self._userInterface = None
        self._users = None
        self._robots = None
        self._actionHistory = None
        self._sensors = None
        self._binary = None
        self._sql = None

    @property
    def locations(self):
        if self._locations is None:
            self._locations = Locations()

        return self._locations

    @property
    def users(self):
        if self._users is None:
            self._users = Users()

        return self._users

    @property
    def robots(self):
        if self._robots is None:
            self._robots = Robots()

        return self._robots

    @property
    def userInterface(self):
        if self._userInterface is None:
            self._userInterface = UserInterface()

        return self._userInterface

    @property
    def actionHistory(self):
        if self._actionHistory is None:
            self._actionHistory = ActionHistory()

        return self._actionHistory

    @property
    def sensors(self):
        if self._sensors is None:
            self._sensors = Sensors()

        return self._sensors

    @property
    def binary(self):
        if self._binary is None:
            self._binary = Binary()

        return self._binary

    @property
    def sql(self):
        if self._sql is None:
            self._sql = SQLDao()

        return self._sql

    def saveRobotLocation(self, robotid, locid, x, y, orientation):
        return self.locations.saveRobotLocation(robotid, locid, x, y, orientation)

    def saveUserLocation(self, userId, locid, x, y, orientation):
        return self.locations.saveUserLocation(userId, locid, x, y, orientation)

    def getActiveQuestion(self):
        return self.userInterface.getActiveQuestion()

    def setResponse(self, sequenceName, responseId):
        return self.userInterface.setResponse(sequenceName, responseId)

    def getResponses(self, sequenceName=None):
        return self.userInterface.getResponses(sequenceName)

    def getLocation(self, locationId):
        return self.locations.getLocation(locationId)

    def getLocationByName(self, name):
        return self.locations.getLocationByName(name)

    def findLocations(self, locationName=None):
        return self.locations.findLocations(locationName)

    def getUser(self, userId):
        return self.users.getUser(userId)

    def getUserByName(self, userName):
        return self.users.getUserByName(userName)

    def findUsers(self, userName=None):
        return self.users.findUsers(userName)

    def getRobot(self, robotId):
        return self.robots.getRobot(robotId)

    def getRobotByName(self, robotName):
        return self.robots.getRobotByName(robotName)

    def findRobots(self, robotName=None):
        return self.robots.findRobots(robotName)

    def getHistory(self, ruleName=None, tags=None):
        return self.actionHistory.getHistory(ruleName, tags)

    def getHistoryOrdered(self, ruleName=None, tags=None):
        return self.actionHistory.getHistoryOrdered(ruleName, tags)

    def getSensor(self, sensorId):
        return self.sensors.getSensor(sensorId)

    def getSensorByname(self, sensorName):
        return self.sensors.getSensorByname(sensorName)

    def findSensors(self, sensorName=None):
        return self.sensors.findSensors(sensorName)

    def getSensorHistory(self, historyId):
        return self.actionHistory.getSensorSnapshot(historyId)

    def saveSensorHistory(self, historyId):
        return self.actionHistory.saveSensorSnapshot(historyId)

    def saveHistory(self, timestamp, ruleName, locationId):
        return self.actionHistory.saveHistory(timestamp, ruleName, locationId)

    def saveHistoryComplete(self, timestamp, ruleName, locationId, user):
        return self.actionHistory.saveHistory(timestamp, ruleName, locationId, user)

    def saveHistoryImage(self, historyId, imageBytes, imageType):
        return self.actionHistory.saveHistoryImage(historyId, imageBytes, imageType)

    def saveHistoryImageOverhead(self, historyId, imageBytes, imageType):
        return self.actionHistory.saveHistoryImageOverhead(historyId, imageBytes, imageType)

    def getBinary(self, itemId):
        return self.binary.getBinary(itemId)

    def getBinaryOverhead(self, itemId):
        return self.binary.getBinaryOverhead(itemId)

    def saveBinary(self, itemName, itemBytes, itemType):
        return self.binary.saveBinary(itemName, itemBytes, itemType)

    def saveBinaryOverhead(self, itemName, itemBytes, itemType):
        return self.binary.saveBinaryOverhead(itemName, itemBytes, itemType)

    def getSingle(self, sql, args=None, default=None):
        return self.sql.getSingle(sql, args, default)

    def getData(self, sql, args=None, trimString=True):
        return self.sql.getData(sql, args, trimString)

    def saveData(self, sql, args=None):
        return self.sql.saveData(sql, args)


class SQLDao(object):

    def __init__(self, hostname=None, username=None, password=None, database=None):
        from config import server_config
        self._host = hostname or server_config['mysql_log_server']
        self._pass = password or server_config['mysql_log_password']
        self._user = username or server_config['mysql_log_user']
        self._db = database or server_config['mysql_log_db']
        self._conn = None

    def __del__(self):
        self.close()

    def getSingle(self, sql, args=None, default=None):
        data = self.getData(sql, args)
        if len(data) > 0:
            return data[0]
        else:
            return default

    def _getCursor(self, retry=10):
        if self._conn is None:
            try:
                self._conn = MySQLdb.connect(self._host, self._user, self._pass, self._db)
            except Exception as e:
                # Access denied error == 1045, no reason to retry
                # Unknown MySQL server host == 2005
                if e.args[0] not in [1045, 2005] and retry > 0:
                    print >> sys.stderr, "Error while connecting: %s, will auto-retry %s more times." % (e, retry)
                    return self._getCursor(retry - 1)
                else:
                    raise e

        try:
            return (self._conn, self._conn.cursor(MySQLdb.cursors.DictCursor))
        except:
            self._conn = None
            return self._getCursor()

    def close(self):
        if self._conn is not None:
            try:
                self._conn.close()
            except:
                pass

            self._conn = None

    def getData(self, sql, args=None, trimString=True):
        (_, cursor) = self._getCursor()

        try:
            cursor.execute(sql, args)
            rows = cursor.fetchall()
            cursor.close()
            return rows
        except Exception as e:

            # Server connection was forcibly severed from the server side
            # retry the request
            if e.args[0] == 2006:
                cursor.close()
                return self.getData(sql, args, trimString)

            if e.args[0] == 2014:
                return self.getData(sql, args, trimString)

            if len(e.args) > 1:
                print "Error %d: %s" % (e.args[0], e.args[1])
            else:
                print "Error %s" & e.args
            return []

    def saveData(self, sql, args=None):
        (conn, cursor) = self._getCursor()

        try:
            cursor.execute(sql, args)
            conn.commit()
            rowId = cursor.lastrowid
            cursor.close()
            # conn.close()
            return rowId
        except MySQLdb.Error, e:
            cursor.close()
            # conn.close()
            # Server connection was forcibly severed from the server side
            # retry the request
            if e.args[0] == 2006:
                return self.saveData(sql, args)

            print "Error %d: %s" % (e.args[0], e.args[1])
            conn.rollback()
            return -1


if __name__ == '__main__':
    h = ActionHistory()
    hi = h.getHistoryByTag('other')
    print hi
