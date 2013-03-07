import MySQLdb, sys

class Locations(object):
    def __init__ (self, robotTable=None, userTable=None, locationTable=None):
        from config import server_config
        self._robotTable = robotTable or server_config['mysql_robot_table']
        self._userTable = userTable or server_config['mysql_users_table']
        self._locationTable = locationTable or server_config['mysql_location_table']
        self._sql = SQLDao()

    def getLocation(self, locationId):
        sql = 'SELECT * FROM `%s`' % (self._locationTable)
        sql += " WHERE `locationId` = %(locid)s" 
        args = {'locid': locationId }

        return self._sql.getSingle(sql, args)

    def getLocationByName(self, name):
        sql = 'SELECT * FROM `%s`' % (self._locationTable)
        sql += " WHERE `name` = %(name)s" 
        args = {'name': name }

        return self._sql.getSingle(sql, args)

    def findLocations(self, locationName=None):
        sql = 'SELECT * FROM `%s`' % (self._locationTable)
        args = None
        if locationName != None:
            sql += " WHERE `locationId` like %(locid)s" 
            args = {'locid': locationName }

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
    
    def __init__ (self, questionTable=None, responseTable=None):
        from config import server_config
        self._guiQuestionTable = questionTable or server_config['mysql_questions_table']
        self._responseTable = responseTable or server_config['mysql_responses_table']
        self._sql = SQLDao()

    def getActiveQuestion(self):
        sql = 'SELECT * FROM `%s`' % (self._guiQuestionTable)
        sql += ' WHERE `guiMsgResult`is NULL' 
        sql += ' ORDER BY `timestamp` desc'
        
        data = self._sql.getSingle(sql)
        if data != None:
            return {
                    'sequenceName' : data['name'],
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
        if sequenceName != None:
            sql += ' WHERE `name` = %(seqName)s'
            args = { 'seqName': sequenceName }
        
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
    def __init__(self, userTable=None, locationTable=None):
        from config import server_config
        self._userTable = userTable or server_config['mysql_users_table']
        self._locationTable = locationTable or server_config['mysql_location_table']
        self._sql = SQLDao()
    
    def getUser(self, userId):
        sql = "SELECT `%(user)s`.*, %(loc)s.name as 'locationName' \
               FROM `%(user)s` \
               INNER JOIN `%(loc)s` ON %(loc)s.`locationId` = `%(user)s`.`locationId`" % {
                              'user': self._userTable,
                              'loc': self._locationTable
                              }
        sql += " WHERE `userId` = %(uid)s"
        args = {'uid': userId }

        return self._sql.getSingle(sql, args)
    
    def getUserByName(self, userName):
        sql = "SELECT `%(user)s`.*, %(loc)s.name as 'locationName' \
               FROM `%(user)s` \
               INNER JOIN `%(loc)s` ON %(loc)s.`locationId` = `%(user)s`.`locationId`" % {
                              'user': self._userTable,
                              'loc': self._locationTable
                              }
        sql += " WHERE `firstName` = %(name)s"
        args = {'name': userName }

        return self._sql.getSingle(sql, args)

    def findUsers(self, userName=None):
        sql = "SELECT `%(user)s`.*, %(loc)s.name as 'locationName' \
               FROM `%(user)s` \
               INNER JOIN `%(loc)s` ON %(loc)s.`locationId` = `%(user)s`.`locationId`" % {
                              'user': self._userTable,
                              'loc': self._locationTable
                              }
        args = None
        if userName != None:
            sql += " WHERE `firstName` like %(name)s"
            args = {'name': userName }

        return self._sql.getData(sql, args)

class Robots(object):
    
    def __init__ (self, robotTable=None, locationTable=None):
        from config import server_config
        self._robotTable = robotTable or server_config['mysql_robot_table']
        self._locationTable = locationTable or server_config['mysql_location_table']
        self._sql = SQLDao()

    def getRobot(self, robotId):
        sql = "SELECT `%(rob)s`.*, %(loc)s.name as 'locationName' \
               FROM `%(rob)s` \
               INNER JOIN `%(loc)s` ON %(loc)s.`locationId` = `%(rob)s`.`locationId`" % {
                              'rob': self._robotTable,
                              'loc': self._locationTable
                              }
        sql += " WHERE `robotid` = %(name)s"
        args = {'name': robotId }

        return self._sql.getSingle(sql, args)
    
    def getRobotByName(self, robotName):
        sql = "SELECT `%(rob)s`.*, %(loc)s.name as 'locationName' \
               FROM `%(rob)s` \
               INNER JOIN `%(loc)s` ON %(loc)s.`locationId` = `%(rob)s`.`locationId`" % {
                              'rob': self._robotTable,
                              'loc': self._locationTable
                              }
        sql += " WHERE `robotName` = %(name)s"
        args = {'name': robotName }

        return self._sql.getSingle(sql, args)

    def findRobots(self, robotName=None):
        sql = "SELECT `%(rob)s`.*, %(loc)s.name as 'locationName' \
               FROM `%(rob)s` \
               INNER JOIN `%(loc)s` ON %(loc)s.`locationId` = `%(rob)s`.`locationId`" % {
                              'rob': self._robotTable,
                              'loc': self._locationTable
                              }
        args = None
        if robotName != None:
            sql += " WHERE `robotName` like %(name)s"
            args = {'name': robotName }

        return self._sql.getData(sql, args)

class ActionHistory(object):
    def __init__(self, actionHistoryTable=None, sensorSnapshotTable=None, sensorTable=None, sensorTypeTable=None, locationTable=None):
        from config import server_config
        self._historyTable = actionHistoryTable or server_config['mysql_history_table']
        self._sensorHistoryTable = sensorSnapshotTable or server_config['mysql_sensorHistory_table']
        self._sensorTable = sensorTable or server_config['mysql_sensor_table']
        self._locationTable = locationTable or server_config['mysql_location_table']
        self._sensorTypeTable = sensorTypeTable or server_config['mysql_sensorType_table']
        self._sql = SQLDao()
        self._selectQuery = "SELECT `actionHistoryId`, `imageId`, `timestamp`, `ruleName`, `tags`, `%(loc)s`.`name` as 'location' \
               FROM `%(hist)s` \
               INNER JOIN `%(loc)s` ON `%(loc)s`.`locationId` = `%(hist)s`.`locationId`" % {
                                                                                            'hist' : self._historyTable,
                                                                                            'loc': self._locationTable
                                                                                           }
    
    def getHistory(self, ruleName=None, tags=None):
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
        return self._processResults(self._sql.getData(sql, args))

    def _appendWhere(self, where, clause):
        if where == None or where == '' or where.strip() == '':
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
                       'location': row['location'],
                       'tags': eval(row['tags'] or '()'),
                       'time': {
                                'real': row['timestamp'].strftime('%Y-%m-%d %H:%M:%S'),
                                'narrative': row['timestamp'].strftime('%Y%m%d%H%M%S')
                                },
                        'sensors': self.getSensorSnapshot(row['actionHistoryId'])
                       }
            
            if type(obj['tags']) != list and type(obj['tags']) != tuple:
                obj['tags'] = (obj['tags'],)
            result.append(obj)
            
        return result

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
        args = { 'histid': historyId}
        
        return self._sql.getData(sql, args)

    def saveSensorSnapshot(self, historyId):
        rowCount = 0
        for row in self.findSensors():
            sensorId = row['sensorId']
            sensorValue = row['value']
                        
            sql = "INSERT INTO `%s` (`actionHistoryId`, `sensorId`, `sensorValue`)" % (self._sensorHistoryTable)
            sql += " VALUES (%(histid)s, %(sensorid)s, %(value)s)"
            args = {
                      'histid' : historyId,
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
                  'time' : timestamp,
                  'rule': ruleName,
                  'locid':locationId
                  }
        
        return self._sql.saveData(sql, args)
    
    def updateTags(self, historyId, tags):
        sql = "UPDATE `%s`" % (self._historyTable)
        sql += " SET `tags` = %(tags)s WHERE `actionHistoryId` = %(histid)s" 
        args = {
              'tags' : str(tags),
              'histid': historyId
              }
        
        self._sql.saveData(sql, args)
        return historyId

    def saveHistoryImage(self, historyId, imageBytes, imageType):
        imageId = self.saveBinary('Image for history %s' % (historyId), imageBytes, imageType)
        sql = "UPDATE `%s`" % (self._historyTable)
        sql += " SET `imageId` = %(imgid)s WHERE `actionHistoryId` = %(histid)s" 
        args = {
              'imgid' : imageId,
              'histid': historyId
              }
        
        self._sql.saveData(sql, args)
        return imageId

class Sensors(object):
    def __init__(self, sensorTable=None, sensorTypeTable=None, sensorLogTable=None, locationTable=None):
        from config import server_config
        self._sensorTable = sensorTable or server_config['mysql_sensor_table']
        self._sensorTypeTable = sensorTypeTable or server_config['mysql_sensorType_table']
        self._sensorLogTable = sensorLogTable or server_config['mysql_log_table']
        self._locationTable = locationTable or server_config['mysql_location_table']
        
        self._sqlQuery = "SELECT `%(sensors)s`.*, `%(type)s`.`sensorType` as `sensorTypeName`, `%(loc)s`.`name` as `locationName` \
                FROM `%(sensors)s` \
                INNER JOIN `%(type)s` ON `%(sensors)s`.`sensorTypeId` = `%(type)s`.`sensorTypeId`\
                INNER JOIN `%(loc)s` ON `%(loc)s`.`locationId` = `%(sensors)s`.`locationId`" % { 
                                                                                 'sensors': self._sensorTable,
                                                                                 'type': self._sensorTypeTable,
                                                                                 'loc': self._locationTable }
        self._sql = SQLDao()
 
    def getSensor(self, sensorId):
        sql = self._sqlQuery
        sql += " WHERE `sensorId` = %(sid)s" 
        args = {'sid': sensorId }
            
        return self._sql.getSingle(sql, args)
    
    def getSensorByName(self, sensorName):
        sql = self._sqlQuery
        sql += " WHERE `name` = %(name)s" 
        args = {'name': sensorName}
            
        return self._sql.getSingle(sql, args)
    
    def findSensors(self, sensorName=None):
        args = None
        sql = self._sqlQuery
        if sensorName != None:
            sql += " WHERE `%s`.`name` like" % (self._sensorTable) + " %(name)s" 
            args = {'name': sensorName}
            
        return self._sql.getData(sql, args)

    def saveSensorLog(self, sensorId, value, status, timestamp, room='', channel=''):
        sql = "INSERT INTO `%(log)s`" % { 'log': self._sensorLogTable }
        sql += "(`timestamp`, `sensorId`, `room`, `channel`, `value`, `status`) \
                    VALUES (%s, %s, %s, %s, %s, %s)"
        
        args = (            timestamp, 
                            sensorId,
                            room, 
                            channel, 
                            value, 
                            status)
        
        if self._sql.saveData(sql, args) >= 0:
            return True
        else:
            return False


class Binary(object):
    def __init__(self, binaryTable=None, binaryIdCol=None):
        from config import server_config
        self._binaryTable = binaryTable or server_config['mysql_image_table']
        self._binaryIdCol = binaryIdCol or 'imageId'
        self._sql = SQLDao()

    def getBinary(self, itemId):
        sql = 'SELECT * FROM `%(table)s` WHERE `%(idCol)s`' % { 'table': self._binaryTable, 'idCol': self._binaryIdCol}        
        sql += " = %(id)s"
        args = {'id': itemId }
        
        data = self._sql.getSingle(sql, args)
        if data != None:
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
                  'type' : itemType,
                  'name': itemName,
                  'bytes':itemBytes
                  }
        
        return self._sql.saveData(sql, args)
    
class DataAccess(object):
    """Obsolete, use individual classes"""
    def __init__ (self, config=None):
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
        if self._locations == None:
            self._locations = Locations()
        
        return self._locations

    @property
    def users(self):
        if self._users == None:
            self._users = Users()
        
        return self._users

    @property
    def robots(self):
        if self._robots == None:
            self._robots = Robots()
        
        return self._robots

    @property
    def userInterface(self):
        if self._userInterface == None:
            self._userInterface = UserInterface()
        
        return self._userInterface

    @property
    def actionHistory(self):
        if self._actionHistory == None:
            self._actionHistory = ActionHistory()
        
        return self._actionHistory

    @property
    def sensors(self):
        if self._sensors == None:
            self._sensors = Sensors()
        
        return self._sensors
    
    @property
    def binary(self):
        if self._binary == None:
            self._binary = Binary()
        
        return self._binary

    @property
    def sql(self):
        if self._sql == None:
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

    def saveHistoryImage(self, historyId, imageBytes, imageType):
        return self.actionHistory.saveHistoryImage(historyId, imageBytes, imageType)
        
    def getBinary(self, itemId):
        return self.binary.getBinary(itemId)
    
    def saveBinary(self, itemName, itemBytes, itemType):
        return self.binary.saveBinary(itemName, itemBytes, itemType)

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
        if self._conn == None:
            try:
                self._conn = MySQLdb.connect(self._host, self._user, self._pass, self._db)
            except Exception as e:
                #TODO: Need to protect this against infinite recursion
                print >> sys.stderr, e
                if retry > 0:
                    return self._getCursor(retry-1)
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
            cursor.close()
            # Server connection was forcibly severed from the server side
            # retry the request
            if e.args[0] == 2006:
                return self.getData(sql, args, trimString)
            
            print "Error %d: %s" % (e.args[0], e.args[1])
            return []
        
    def saveData(self, sql, args=None):
        (conn, cursor) = self._getCursor()

        try:
            cursor.execute(sql, args)
            conn.commit()
            rowId = cursor.lastrowid
            cursor.close()
            #conn.close()
            return rowId
        except MySQLdb.Error, e:
            cursor.close()
            #conn.close()
            # Server connection was forcibly severed from the server side
            # retry the request
            if e.args[0] == 2006:
                return self.saveData(sql, args)
            
            print "Error %d: %s" % (e.args[0], e.args[1])            
            conn.rollback()
            return -1


if __name__ == '__main__':
    h = ActionHistory()
    hi =h.getHistoryByTag('other')
    print hi