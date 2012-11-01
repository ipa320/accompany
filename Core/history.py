import datetime
from Data.dataAccess import DataAccess
from threading import Thread
from extensions import PollingThread
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

if __name__ == '__main__':
    #import sys
    #print sys.argv
    #h = ActionHistory()
    #print h.addHistory('testRule')
