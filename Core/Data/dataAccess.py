import MySQLdb

class DataAccess(object):
    def __init__ (self, config=None):
        if config == None:
            from config import server_config
            config = server_config

        self._binaryTable = config['mysql_image_table']
        self._binaryIdCol = 'imageId'
        self._historyTable = config['mysql_history_table']
        self._locationTable = config['mysql_location_table']
        self._robotTable = config['mysql_robot_table']
        self._sensorHistoryTable = config['mysql_sensorHistory_table']
        self._sensorTable = config['mysql_sensor_table']
        self._sensorTypeTable = config['mysql_sensor_table']
        self._guiQuestionTable = config['mysql_questions_table']
        self._responseTable = config['mysql_responses_table']
        self._userTable = config['mysql_users_table']
        self._config = config

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
        
        return self.saveData(sql, args) >= 0

    def saveRobotLocation(self, robotid, locid, x, y, orientation):
        return self.saveLocation(robotid, locid, x, y, orientation, self._robotTable, 'robotId')
        
    def saveUserLocation(self, userId, locid, x, y, orientation):
        return self.saveLocation(userId, locid, x, y, orientation, self._userTable, 'userId')

    def getActiveQuestion(self):
        sql = 'SELECT * FROM `%s`' % (self._guiQuestionTable)
        sql += ' WHERE `guiMsgResult`is NULL' 
        sql += ' ORDER BY `timestamp` desc'
        
        data = self.getSingle(sql)
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
        
        return self.saveData(sql, args) >= 0
        
    def getResponses(self, sequenceName=None):
        sql = 'SELECT * FROM `%s`' % (self._responseTable)
        args = None
        if sequenceName != None:
            sql += ' WHERE `name` = %(seqName)s'
            args = { 'seqName': sequenceName }
        
        data = []
        sizer = lambda x: 'l' if int(x) > 0 else 's'
        for response in self.getData(sql, args):
            for msgId in range(1, 5):
                if len(response['guiMsg%s' % (msgId)].strip()) > 0:
                    data.append({
                                 'guiResponseId':msgId,
                                 'message':response['guiMsg%s' % (msgId)],
                                 'size': sizer(response['guiMsg%sEnlarged' % (msgId)]),
                                 'seqName':response['name']
                                 })
        return data

    def getLocation(self, locationId):
        sql = 'SELECT * FROM `%s`' % (self._locationTable)
        sql += " WHERE `locationId` = %(locid)s" 
        args = {'locid': locationId }

        return self.getSingle(sql, args)

    def getLocationByName(self, name):
        sql = 'SELECT * FROM `%s`' % (self._locationTable)
        sql += " WHERE `name` = %(name)s" 
        args = {'name': name }

        return self.getSingle(sql, args)

    def findLocations(self, locationName=None):
        sql = 'SELECT * FROM `%s`' % (self._locationTable)
        args = None
        if locationName != None:
            sql += " WHERE `locationId` like %(locid)s" 
            args = {'locid': locationName }

        return self.getData(sql, args)
    
    def getUser(self, userId):
        sql = "SELECT `%(user)s`.*, %(loc)s.name as 'locationName' \
               FROM `%(user)s` \
               INNER JOIN `%(loc)s` ON %(loc)s.`locationId` = `%(user)s`.`locationId`" % {
                              'user': self._userTable,
                              'loc': self._locationTable
                              }
        sql += " WHERE `userId` = %(uid)s"
        args = {'uid': userId }

        return self.getSingle(sql, args)
    
    def getUserByName(self, userName):
        sql = "SELECT `%(user)s`.*, %(loc)s.name as 'locationName' \
               FROM `%(user)s` \
               INNER JOIN `%(loc)s` ON %(loc)s.`locationId` = `%(user)s`.`locationId`" % {
                              'user': self._userTable,
                              'loc': self._locationTable
                              }
        sql += " WHERE `userName` = %(name)s"
        args = {'name': userName }

        return self.getSingle(sql, args)

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

        return self.getData(sql, args)
    
    def getRobot(self, robotId):
        sql = "SELECT `%(rob)s`.*, %(loc)s.name as 'locationName' \
               FROM `%(rob)s` \
               INNER JOIN `%(loc)s` ON %(loc)s.`locationId` = `%(rob)s`.`locationId`" % {
                              'rob': self._robotTable,
                              'loc': self._locationTable
                              }
        sql += " WHERE `robotid` = %(name)s"
        args = {'name': robotId }

        return self.getSingle(sql, args)
    
    def getRobotByName(self, robotName):
        sql = "SELECT `%(rob)s`.*, %(loc)s.name as 'locationName' \
               FROM `%(rob)s` \
               INNER JOIN `%(loc)s` ON %(loc)s.`locationId` = `%(rob)s`.`locationId`" % {
                              'rob': self._robotTable,
                              'loc': self._locationTable
                              }
        sql += " WHERE `robotName` = %(name)s"
        args = {'name': robotName }

        return self.getSingle(sql, args)

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

        return self.getData(sql, args)

    def getHistory(self, ruleName=None):
        sql = "SELECT `actionHistoryId`, `imageId`, `timestamp`, `ruleName`, `%(loc)s`.`name` as 'location' \
               FROM `%(hist)s` \
               INNER JOIN `%(loc)s` ON `%(loc)s`.`locationId` = `%(hist)s`.`locationId`" % {
                                                                                            'hist' : self._historyTable,
                                                                                            'loc': self._locationTable
                                                                                           }
        args = None
        if ruleName != None:
            sql += " WHERE `ruleName` = %(name)s" 
            args = {'name': ruleName}

        obj = []
        for row in self.getData(sql, args):
            obj.append({
                       'id': row['actionHistoryId'],
                       'name':row['ruleName'],
                       'status': 'activate',
                       'imageId':row['imageId'],
                       'location': row['location'],
                       'time': {
                                'real': row['timestamp'].strftime('%Y-%m-%d %H:%M:%S'),
                                'narrative': row['timestamp'].strftime('%Y%m%d%H%M%S')
                                },
                        'sensors': self.getSensorHistory(row['actionHistoryId'])
                       })
        
        return obj
    
    def getSensor(self, sensorId):
        sql = "SELECT * FROM `%s`" % (self._sensorTable)
        sql += " WHERE `sensorId` = %(sid)s" 
        args = {'sid': sensorId}
            
        return self.getSingle(sql, args)
    
    def getSensorByName(self, sensorName):
        sql = "SELECT * FROM `%s`" % (self._sensorTable)
        sql += " WHERE `name` = %(name)s" 
        args = {'name': sensorName}
            
        return self.getSingle(sql, args)
    
    def findSensors(self, sensorName=None):
        sql = "SELECT * FROM `%s`" % (self._sensorTable)
        
        args = None
        if sensorName != None:
            sql += " WHERE `name` like %(name)s" 
            args = {'name': sensorName}
            
        return self.getData(sql, args)

    def getSensorHistory(self, historyId):
        sql = "SELECT `%(sensor)s`.`name` as `name`, `%(sensor)s`.`sensorId` as `sensorId`, `%(sensor)s`.`sensorRule` as `sensorRule`, `%(loc)s`.`name` as `location`, `%(hist)s`.`sensorValue` as `value`\
        FROM `%(hist)s` \
        INNER JOIN `%(sensor)s` ON `%(sensor)s`.`sensorId` = `%(hist)s`.`sensorId`\
        INNER JOIN `%(loc)s` ON `%(loc)s`.`locationId` = `%(sensor)s`.`locationId` " % {
                                          'hist': self._sensorHistoryTable,
                                          'loc': self._locationTable,
                                          'sensor': self._sensorTable
                                          }        
        sql += " WHERE `actionHistoryId` = %(histid)s"        
        args = { 'histid': historyId}
        
        return self.getData(sql, args)

    def saveSensorHistory(self, historyId):
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

            if self.saveData(sql, args) >= 0:
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
        
        return self.saveData(sql, args)

    def saveHistoryImage(self, historyId, imageBytes, imageType):
        imageId = self.saveBinary('Image for history %s' % (historyId), imageBytes, imageType)
        sql = "UPDATE `%s`" % (self._historyTable)
        sql += " SET `imageId` = %(imgid)s WHERE `actionHistoryId` = %(histid)s" 
        args = {
              'imgid' : imageId,
              'histid': historyId
              }
        
        self.saveData(sql, args)
        return imageId
        
    def getBinary(self, itemId):
        sql = 'SELECT * FROM `%(table)s` WHERE `%(idCol)s`' % { 'table': self._binaryTable, 'idCol': self._binaryIdCol}        
        sql += " = %(id)s"
        args = {'id': itemId }
        
        data = self.getSingle(sql, args)
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
        
        return self.saveData(sql, args)

    def getSingle(self, sql, args=None, default=None):
        data = self.getData(sql, args)
        if len(data) > 0:
            return data[0]
        else:
            return default

    def _getCursor(self):
        #get a new connection for each request
        conn = MySQLdb.connect(self._config['mysql_log_server'],
                                        self._config['mysql_log_user'],
                                        self._config['mysql_log_password'],
                                        self._config['mysql_log_db'])
        
        return (conn, conn.cursor(MySQLdb.cursors.DictCursor))

    def getData(self, sql, args=None, trimString=True):
        (conn, cursor) = self._getCursor()

        try:
            cursor.execute(sql, args)
            rows = cursor.fetchall()
            cursor.close()
            conn.close()
            return rows
        except Exception as e:
            cursor.close()
            conn.close()
            #Server connection was forcably severed from the server side
            #retry the request
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
            conn.close()
            return rowId
        except MySQLdb.Error, e:
            cursor.close()
            conn.close()
            #Server connection was forcably severed from the server side
            #retry the request
            if e.args[0] == 2006:
                return self.saveData(sql, args)
            
            print "Error %d: %s" % (e.args[0], e.args[1])            
            conn.rollback()
            return -1
