import datetime
from Data.dataAccess import DataAccess
from threading import Thread
from extensions import PollingThread, PollingProcessor

""" Classes that are responsible for storing history information in the database """

class ActionHistory(object):
    """ Handles action history requests """
    _defaultImageType = 'png'
    _runningThreads = {}
    
    def cancelPollingHistory(self, ruleName):
        """ Stops the named polling history thread """
        if ActionHistory._runningThreads.has_key(ruleName):
            ah = ActionHistory._runningThreads[ruleName]
            ah.cancel()
            ah.join()
            return True
        else:
            return False

    def addPollingHistory(self, ruleName, delaySeconds):
        """ Starts automatically polling and updating the action history table every x seconds with the specified rule name """
        if not ActionHistory._runningThreads.has_key(ruleName):
            ahw = PollingThread(target=self.addHistory, delayTime=delaySeconds, args=(ruleName,), completeCallback=self._removePollingHistory)
            ahw.start()
            ActionHistory._runningThreads[ruleName] = ahw
        
        return ruleName
    
    def _removePollingHistory(self, ruleName):
        return ActionHistory._runningThreads.pop(ruleName, None)

    def addHistoryAsync(self, ruleName, imageBytes=None, imageType=None):
        """ asynchronously updates the actionHistory table, returning immediately """  
        Thread(target=self.addHistory, args=(ruleName, imageBytes, imageType)).start()

    def addHistory(self, ruleName, imageBytes=None, imageType=None):
        """ updates the action history table, blocking until all data is retrieved and stored """
        """ returns true on successful update """
        
        from Robots.robotFactory import Factory
        cob = Factory.getCurrentRobot()
        dao = DataAccess()
        dateNow = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        location = dao.getRobotByName(cob.name)['locationId']
        
        historyId = dao.saveHistory(dateNow, ruleName, location)
        
        if(historyId > 0):
            dao.saveSensorHistory(historyId)

            if imageType == None:
                imageType = ActionHistory._defaultImageType

            if imageBytes == None:
                imageBytes = cob.getImage(retFormat=imageType)

            if imageBytes != None:
                dao.saveHistoryImage(historyId, imageBytes, imageType)
        
        return historyId > 0

    def addHistoryCompleteAsync(self, ruleName, imageBytes=None, imageType=None, imageOverheadBytes=None, imageOverheadType=None):
        """ asynchronously updates the actionHistory table, returning immediately """  
        Thread(target=self.addHistoryComplete, args=(ruleName, imageBytes, imageType,imageOverheadBytes,imageOverheadType)).start()

    def addHistoryComplete(self, ruleName, imageBytes=None, imageType=None, imageOverheadBytes=None, imageOverheadType=None):
        """ updates the action history table, blocking until all data is retrieved and stored """
        """ returns true on successful update """
        
        from Robots.robotFactory import Factory
        cob = Factory.getCurrentRobot()
        dao = DataAccess()
        dateNow = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        location = dao.getRobotByName(cob.name)['locationId']
        
        historyId = dao.saveHistory(dateNow, ruleName, location)
        
        if(historyId > 0):
            dao.saveSensorHistory(historyId)

            if imageType == None:
                imageType = ActionHistory._defaultImageType

            if imageBytes == None:
                imageBytes = cob.getImage(retFormat=imageType)

            if imageBytes != None:
                dao.saveHistoryImage(historyId, imageBytes, imageType)

            #the same but for the overhead camera image

            if imageOverheadType == None:
                imageOverheadType = ActionHistory._defaultImageType

            if imageOverheadBytes == None:
                imageOverheadBytes = cob.getImageOverhead(retFormat=imageOverheadType)

            if imageOverheadBytes != None:
                dao.saveHistoryImageOverhead(historyId, imageOverheadBytes, imageOverheadType)
        
        return historyId > 0



################################################################################
#
# Logger thread
#
# Logs channel value and / or status changes into a (separate) MySQL
# database table.
#
################################################################################
class SensorLog(PollingProcessor):
    """ Handles updating sensors of all types to the database """
    def __init__ (self, channels, name=''):
        super(SensorLog, self).__init__()
        self._dao = DataAccess().sensors
        self._channels = channels
        self._logCache = {}
        self._name = name
                
    def start(self):
        """ Begin asynchronously updating changes in sensors """
        if self._name != '':
            print "Started updating database for %s sensor changes" % (self._name)
        else:
            print "Started updating database for [unknown] sensor changes"
        self._addPollingProcessor('sensorHistory', self.checkUpdateSensors, (self._channels,), 0.01)

    def stop(self):
        """ Stop updating the sensor table """
        if self._name != '':
            print "Stopped updating database for %s sensor changes" % (self._name)
        else:
            print "Stopped updating database for [unknown] sensor changes"

        self._removePollingProcessor('sensorHistory')

    def checkUpdateSensors(self, channels):
        """ Check the specified channels and update the database whenever changes are detected to the 'value' or 'status'"""
        for uuid, sensor in channels.items():
            if not self._logCache.has_key(uuid):
                current = self._dao.getSensor(sensor['id'])
                self._logCache[uuid] = { 'value': current['value'], 'status': current['status']}

            status = str(sensor['status']).capitalize()
            if self._logCache[uuid]['status'] != status or self._logCache[uuid]['value'] != sensor['value']:
                val = sensor['value'] if len(sensor['value']) > 1 else sensor['value'][0]
                timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                success = self._dao.saveSensorLog(
                                                  sensor['id'],
                                                  val,
                                                  status,
                                                  timestamp,
                                                  sensor['room'],
                                                  sensor['channel'])
                if success:
                    print "Updated sensor log for %(id)s to %(status)s (%(value)s)" % { 
                                                                           'id':sensor['channel'],
                                                                           'status': status,
                                                                           'value': val,
                                                                           }
                    self._logCache[uuid]['value'] = sensor['value']
                    self._logCache[uuid]['status'] = status

if __name__ == '__main__':
    import sys
    print >> sys.stderr, "Sensor update code has moved to sensor.py"
    print >> sys.stderr, "Run 'python sensors.py' to begin monitoring sensors"
