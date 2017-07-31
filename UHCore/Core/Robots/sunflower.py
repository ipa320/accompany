import math
from robot import ROSRobot

class Sunflower(ROSRobot):
    """ Concrete implementation of the Robot interface for the UH Sunflower model of robots """
    _imageFormats = ['BMP', 'EPS', 'GIF', 'IM', 'JPEG', 'PCD', 'PCX', 'PDF', 'PNG', 'PPM', 'TIFF', 'XBM', 'XPM']

    def __init__(self, name, rosMaster):
        from rosHelper import ROS
        ROS.configureROS(rosMaster=rosMaster)
        super(Sunflower, self).__init__(name, ActionLib, 'sf_controller', '')

    def setComponentState(self, name, value):
        # check if the component has been initialised, and init if it hasn't
        if name == 'base':
            self._robInt.initComponent(name)

        return super(Sunflower, self).setComponentState(name, value)

class ActionLib(object):
        
    def __init__(self):
        import rosHelper
        self._ros = rosHelper.ROS()
        self._ros.configureROS(packageName='sf_controller')
        self._ros.configureROS(packageName='sf_lights')
        
        import actionlib, sf_controller.msg, sf_lights.msg
        self._sfMsgs = sf_controller.msg
        self._sfLights = sf_lights.msg
        
        self._ros.initROS()
        self._sfClient = actionlib.SimpleActionClient('/sf_controller', self._sfMsgs.SunflowerAction)
        print "Waiting for sf_controller..."
        self._sfClient.wait_for_server()
        print "Connected to sf_controller"

        self._sfLight = actionlib.SimpleActionClient('/lights', self._sfLights.LightsAction)
        print "Waiting for sf_lights..."
        self._sfLight.wait_for_server()
        print "Connected to sf_lights"
        
    def runFunction(self, funcName, kwargs):
        return 5
        
    def initComponent(self, name):
        if name == 'base':
            goal = self._sfMsgs.SunflowerGoal(
                                                   action='init',
                                                   component=name)
            client = self._sfClient
            return client.send_goal_and_wait(goal)
        else:
            return 3
    
    def runComponent(self, name, value, mode=None, blocking=True):
        if name == 'light':
            goal = self._sfLights.LightsGoal(rgb=value)
            client = self._sfLight
        else:
            (namedPosition, joints) = (value, []) if str == type(value) else ('', value)
            
            goal = self._sfMsgs.SunflowerGoal(
                                                   action='move',
                                                   component=name,
                                                   namedPosition=namedPosition,
                                                   jointPositions=joints)
            client = self._sfClient

        if(blocking):
            status = client.send_goal_and_wait(goal)
        else:
            client.send_goal(goal)
            status = 1
            
        return status
    
if __name__ == '__main__':
    s = Sunflower()
    print "Light red"
    s.setLight([1, 0, 0])
    
    # joint_names: ["head_pan", "head_tilt", "neck_upper", "neck_lower"]
    # print "Head home " + s.setComponentState('head', 'home')
    print "Head back " + s.setComponentState('head',
                               [ math.radians(90),
                                 math.radians(45),
                                 math.radians(90),
                                 math.radians(-90)])
    # print s.setComponentState('base', [0,0, math.radians(-15)])
    print "Head back " + s.setComponentState('head',
                               [ math.radians(-90),
                                 math.radians(-45),
                                 math.radians(90),
                                 math.radians(-90)])
    # print s.setComponentState('base', [0,0, math.radians(15)])
    print "Head home " + s.setComponentState('head', 'home')
    print "Light green"
    s.setLight([0, 1, 0])

    print "Tray open " + s.setComponentState('tray', 'open')
    print "Tray close " + s.setComponentState('tray', [0])
    print "Light off"
    s.setLight([0, 0, 0])
    
