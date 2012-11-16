import datetime
from Data.dataAccess import DataAccess, SQLDao
from threading import Thread
from extensions import PollingThread, PollingProcessor
from Robots.careobot import CareOBot

class ActionHistory(object):
    _defaultImageType = 'png'
    _runningThreads = {}
    
    def cancelPollingHistory(self, ruleName):
        if ActionHistory._runningThreads.has_key(ruleName):
            ah = ActionHistory._runningThreads[ruleName]
            ah.cancel()
            ah.join()
            return True
        else:
            return False

    def addPollingHistory(self, ruleName, delaySeconds):
        if not ActionHistory._runningThreads.has_key(ruleName):
            ahw = PollingThread(target=self.addHistory, delayTime=delaySeconds, args=(ruleName,), completeCallback=self._removePollingHistory)
            ahw.start()
            ActionHistory._runningThreads[ruleName] = ahw
        
        return ruleName
    
    def _removePollingHistory(self, ruleName):
        return ActionHistory._runningThreads.pop(ruleName, None)

    def addHistoryAsync(self, ruleName, imageBytes=None, imageType=None):
        Thread(target=self.addHistory, args=(ruleName, imageBytes, imageType)).start()

    def addHistory(self, ruleName, imageBytes=None, imageType=None):
        
        cob = CareOBot()
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

################################################################################
#
# Logger thread
#
# Logs channel value and / or status changes into a (separate) MySQL
# database table.
#
################################################################################
class SensorHistory(PollingProcessor):    
    def __init__ (self, channelFunc):
        super(SensorHistory, self).__init__()
        self._dao = SQLDao()
        self._channelFunc = channelFunc
                
    # This method handles the actual writing procedure.
    def writechannel(self, channel):
        dateNow = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        sql = "INSERT INTO `%s` (`timestamp`, `sensorId`, `room`, `channel`, `value`, `status`) VALUES ('%s', '%s', '%s', '%s', '%s', '%s')"
        args = (            dateNow, 
                            channel['id'],
                            channel['room'], 
                            channel['channel'], 
                            channel['value'], 
                            channel['status'])
        
        if self._dao.saveData(sql, args) > 0:
            return True
        else:
            return False

    def startHistoryPolling(self):
        print "Started polling sensor changes"
        self._addPollingProcessor('sensorHistory', self.checkUpdateSensors, (self._channelFunc), 0.01)

    def stopHistoryPolling(self):
        print "Stopped polling sensor changes"
        self._removePollingProcessor('sensorHistory')

    def checkUpdateSensors(self, channelFunc):
        logged_channels = {}

        channels = channelFunc()
        key = channels.keys()
        for k in key:
            try:
#               if logged_channels[k]['value'] != channels[k]['value'] or 
                if logged_channels[k]['status'] != channels[k]['status']:
                    self.writechannel(channels[k])
                    logged_channels[k]['value'] = channels[k]['value']
                    logged_channels[k]['status'] = channels[k]['status']
            except:
                self.writechannel(channels[k])
                logged_channels.setdefault(k, {})
                logged_channels[k].setdefault('value', channels[k]['value'])
                logged_channels[k].setdefault('status', channels[k]['status'])
