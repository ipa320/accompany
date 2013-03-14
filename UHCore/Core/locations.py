from Data.dataAccess import DataAccess
from extensions import PollingProcessor
import math

class LocationProcessor(PollingProcessor):
    
    def __init__(self):
        super(LocationProcessor, self).__init__()
        self._dao = DataAccess()
        self._storedLoc = lambda: {'locationName': '', 'xCoord': 0, 'yCoord': 0, 'orientation': 0, 'driftThreshold': 0 }
        self._curLoc = lambda: ('', (0, 0, 0))
        self._updateLoc = lambda locid, x, y, orientation: True
        self._targetName = ''
    
    def start(self):
        print "Started polling location for %s" % (self._targetName)
        self._addPollingProcessor('location', self.checkUpdateLocation, (self._curLoc, self._storedLoc, self._updateLoc), 2)

    def stop(self):
        print "Stopped polling location for %s" % (self._targetName)
        self._removePollingProcessor('location')
    
    def checkUpdateLocation(self, getCurrentLocation, getSavedLocation, updateLocation):
        savedLoc = getSavedLocation()
        (name, (x, y, orientation)) = getCurrentLocation()

        if x == None or y == None:
            return
        
        update = False
        if name != savedLoc['locationName']:
            update = True
        else:
            dist = 0
            dist += math.pow(x - savedLoc['xCoord'], 2)
            dist += math.pow(y - savedLoc['yCoord'], 2)
            dist = math.sqrt(dist)
            if not savedLoc.has_key('driftThreshold') or dist > savedLoc['driftThreshold']:
                update = True
            else:
                if abs(savedLoc['orientation'] - orientation) > 5:                     
                    update = True

        if update:
            loc = self._dao.getLocationByName(name)
            if loc == None:
                locid = None
            else:
                locid = loc['locationId']
            print "Updating %(target)s location to Name:%(name)s, X:%(x)s, Y:%(y)s, O:%(orientation)s" % dict({'target':self._targetName}.items() + locals().items())
            updateLocation(locid, x, y, orientation)

class RobotLocationProcessor(LocationProcessor):
    
    def __init__(self, robot=None):
        super(RobotLocationProcessor, self).__init__()
        if robot == None:
            from Robots.careobot import CareOBot
            self._robot = CareOBot()
        else:
            self._robot = robot

        robId = self._dao.getRobotByName(self._robot.name)['robotId']
        self._targetName = self._robot.name
        self._storedLoc = lambda: self._dao.getRobot(robId)
        self._curLoc = lambda: self._robot.getLocation(False)
        
        self._updateLoc = lambda locid, x, y, orientation: self._dao.saveRobotLocation(robId, locid, x, y, orientation)

class HumanLocationProcessor(LocationProcessor):
    
    def __init__(self, robot=None):
        super(HumanLocationProcessor, self).__init__()
        if robot == None:
            from Robots.careobot import CareOBot
            self._robot = CareOBot()
        else:
            self._robot = robot

        robId = self._dao.getRobotByName(self._robot.name)['robotId']
        self._targetName = self._robot.name
        self._storedLoc = lambda: self._dao.getRobot(robId)
        self._curLoc = lambda: self._robot.getLocation(False)
        
        self._updateLoc = lambda locid, x, y, orientation: self._dao.saveRobotLocation(robId, locid, x, y, orientation)


if __name__ == '__main__':
    import sys
    lp = RobotLocationProcessor()
    lp.start()
    while True:
        try:
            sys.stdin.read()
        except KeyboardInterrupt:
            break
    lp.stop()