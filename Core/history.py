import datetime
from Data.dataAccess import DataAccess
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
class SensorLog(PollingProcessor):    
    def __init__ (self, channels, name=''):
        super(SensorLog, self).__init__()
        self._dao = DataAccess().sensors
        self._channels = channels
        self._logCache = {}
        self._name = name
                
    def start(self):
        if self._name != '':
            print "Started polling sensor changes for %s" % (self._name)
        else:
            print "Started polling sensor changes"
        self._addPollingProcessor('sensorHistory', self.checkUpdateSensors, (self._channels, ), 0.01)

    def stop(self):
        if self._name != '':
            print "Stopped polling sensor changes for %s" % (self._name)
        else:
            print "Stopped polling sensor changes"

        self._removePollingProcessor('sensorHistory')

    def checkUpdateSensors(self, channels):
        for k in channels.keys():
            if not self._logCache.has_key(k):
                current = self._dao.getSensor(channels[k]['id'])
                self._logCache.setdefault(k, { 'value': current['value'], 'status': current['status']})
            if self._logCache[k]['status'] != channels[k]['status']:
                timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                success = self._dao.saveSensorLog(
                                                  channels[k]['id'], 
                                                  channels[k]['value'], 
                                                  channels[k]['status'],
                                                  timestamp,
                                                  channels[k]['room'],
                                                  channels[k]['channel'])
                if success:
                    print "Updated sensor log for %(id)s to %(status)s" % { 
                                                                           'id':channels[k]['channel'], 
                                                                           'status': channels[k]['status']
                                                                           }
                    self._logCache[k]['value'] = channels[k]['value']
                    self._logCache[k]['status'] = channels[k]['status']
        

if __name__ == '__main__':
    import sys
    import config
    import sensors
    z = sensors.ZigBee(config.server_config['udp_listen_port'])
    g = sensors.GEOSystem(config.server_config['mysql_geo_server'],
                            config.server_config['mysql_geo_user'],
                            config.server_config['mysql_geo_password'],
                            config.server_config['mysql_geo_db'],
                            config.server_config['mysql_geo_query'])
    sz = SensorLog(z.channels)
    sg = SensorLog(g.channels)
    z.start()
    g.start()
    sz.start()
    sg.start()
    while True:
        try:
            sys.stdin.read()
        except KeyboardInterrupt:
            break
    sz.stop()
    sg.stop()
    g.stop()
    z.stop()