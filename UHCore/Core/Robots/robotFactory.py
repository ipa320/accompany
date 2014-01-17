from Data.dataAccess import Robots, Locations
import sys
from config import robot_config

class Factory(object):
    """Factory methods for creating Robot classes"""

    @staticmethod
    def getCurrentRobot():
        """ Retrieves the current Robot type from the database, given the active experiment location from the """
        """ session control table and constructs the appropriate class """ 
        activeLocation = Locations().getActiveExperimentLocation()
        if activeLocation == None:
            print "No experiment location set"
        elif activeLocation['activeRobot'] != None and activeLocation['activeRobot'] != '':
            robotData = Robots().getRobot(activeLocation['activeRobot'])
            if robotData != None and robotData['robotName'] != '':
                return Factory.getRobot(robotData['robotName'])
            else:
                print >> sys.stderr, "No data retrieved for robot ID %s" % activeLocation['activeRobot']
        else:
            print >> sys.stderr, "No robots set for experiment location %s" % activeLocation['location']
        
        return None
        
    @staticmethod
    def getRobot(robotName=None):
        """ Retrieves the named robot type from the database and constructs the appropriate class """
        """ if robotName is None or Empty, acts like getCurrentRobot() """
        if robotName == None or robotName == '':
            return Factory.getCurrentRobot()
        
        print "Building class for robot named: %s" % robotName
        robot = None
        if robotName.lower().startswith('care-o-bot'):
            import careobot
            if robot_config.has_key(robotName) and robot_config[robotName].has_key('hostname'):
                rosMaster = 'http://%s:11311'
            else:
                cobVersion = robotName[11:].replace('.', '-')
                rosMaster = "http://cob%s-pc1:11311" % cobVersion
            robot = careobot.CareOBot(robotName, rosMaster)
        elif robotName.lower().startswith('sunflower'):
            import sunflower
            if robot_config.has_key(robotName) and robot_config[robotName].has_key('hostname'):
                rosMaster = 'http://%s:11311'
            else:
                cobVersion = robotName[10:].replace('.', '-')
                rosMaster = "http://sunflower%s-pc1:11311" % cobVersion
            robot = sunflower.Sunflower(robotName, rosMaster)
        elif robotName.lower().startswith('dummy'):
            import dummy
            robot = dummy.DummyRobot(robotName)
        else:
            print >> sys.stderr, "Unknown robot %s" % robotName
            return None

        print "Finished building class %s" % robot.__class__.__name__
        return robot
