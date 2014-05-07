#!/usr/bin/env python
import math, sys
import time
import robot
import rosHelper
import inspect
from config import robot_config
from collections import deque

class CareOBot(robot.ROSRobot):
    " Concrete implementation of Robot interface for IPA Care-O-Bot robots"
    
    _imageFormats = ['BMP', 'EPS', 'GIF', 'IM', 'JPEG', 'PCD', 'PCX', 'PDF', 'PNG', 'PPM', 'TIFF', 'XBM', 'XPM']

    def __init__(self, name, rosMaster):
        rosHelper.ROS.configureROS(rosMaster=rosMaster)
        super(CareOBot, self).__init__(name, ActionLib, 'script_server', robot_config[name]['head']['camera']['topic'])
        #super(CareOBot, self).__init__(name, ScriptServer, 'script_server', robot_config[name]['head']['camera']['topic'])
               
    def getCameraAngle(self):
        "Return the vertical component of the camera angle, used by getImage for rotating the image when needed"
        state = self.getComponentState('head', True)
        if state == None:
            return None
        else:
            (_, cameraState) = state
        
        pos = cameraState['positions'][0]
        angle = round(math.degrees(pos), 2)
        angle = angle % 360
            
        return angle
                
    def setComponentState(self, name, value, blocking=True):
        # check if the component has been initialised, and init if it hasn't
        if len(self._ros.getTopics('/%(name)s_controller' % { 'name': name })) == 0:
            self._robInt.initComponent(name)
        
        #Special handling of the unload_tray moveit code value='trayToTable:height'
        if name == 'arm' and str(value).startswith('trayToTable'):
            return self.unloadTray(str(value).split(':')[1], blocking)
        
        return super(CareOBot, self).setComponentState(name, value, blocking)
    
    def play(self, fileName, blocking=True):
        self.executeFunction("play", {
                                      'parameter_name':fileName,
                                      'blocking':blocking
                                      })
    
    def say(self, text, languageCode="en-gb", blocking=True):
        self.executeFunction("say", {
                                     'parameter_name': [text,],
                                     'blocking': blocking })
        
    def sleep(self, milliseconds):
        self.executeFunction("sleep", {'duration': milliseconds / 1000.0 })
        
    def unloadTray(self, height, blocking):
        " Calls the 'unloadTrayServer' on to unload an object from the tray to a table assumed to be at the specified height"

        try:
            h = float(height)
        except Exception as e:
            print >> sys.stderr, "Unable to cast height to float, received height: %s" % height
            
        try:
            client = UnloadTrayClient()
        except Exception as e:
            print >> sys.stderr, "Unable to initialise UnloadTrayClient. Error: %s" % repr(e)
            return robot._states[4]
        
        return client.unloadTray(h, blocking)
        
class UnloadTrayClient(object):
    
    def __init__(self):
        self._ros = rosHelper.ROS()
        self._ros.configureROS(packageName='accompany_user_tests_year2')
        import actionlib, accompany_user_tests_year2.msg
        self._ssMsgs = accompany_user_tests_year2.msg
        
        self._ros.initROS()
        self._client = actionlib.SimpleActionClient('/unload_tray', self._ssMsgs.UnloadTrayAction)
        print "Waiting for unload_tray"
        self._client.wait_for_server()
        print "Connected to unload_tray"
        
    def unloadTray(self, height, blocking):
        goal = self._ssMsgs.UnloadTrayGoal()
        goal.table_height = height
        
        if blocking:
            return robot._states[self._client.send_goal_and_wait(goal)]
        else:
            return robot._states[self._client.send_goal(goal)]
    
class ScriptServer(object):
    """ Concrete implementation of the robotInterface class using direct calls to simple_script_server """
    _specialCases = {
                    'light': {'function': 'set_light', 'mode': ''},
                    'sound': {'function': 'say', 'mode': 'FEST_EN' }
                    }
    
    def __init__(self):
        self._ros = rosHelper.ROS()
        self._ros.configureROS(packageName='cob_script_server')
        import simple_script_server
        self._ros.initROS()
        self._ss = simple_script_server.simple_script_server()
    
    def runFunction(self, funcName, kwargs):
        try:
            func = getattr(self._ss, funcName)
        except AttributeError:
            raise Exception('Unknown function: %s' % (funcName))
        
        #filter kwargs to only args in function
        argspec = inspect.getargspec(func)
        args = {}
        for argName in argspec.args:
            if argName in kwargs.keys():
                args[argName] = kwargs[argName]
        #args = { argName: kwargs[argName] for argName in argspec.args if argName in kwargs.keys() }

        ah = func(**args)
        
        if funcName != 'sleep':
            return ah.get_state()
    
    def stopComponent(self, name):
        if name in ScriptServer._specialCases:
            return 3

        return self.runFunction("stop", { 'component_name':name, 'blocking':False })
    
    def initComponent(self, name):       
        if name not in ScriptServer._specialCases.keys():
            return self.runFunction('init', {'component_name': name} )
        return 3
    
    def runComponent(self, name, value, mode='', blocking=True):
        if name not in ScriptServer._specialCases.keys():
            func = "move"
        else:
            func = ScriptServer._specialCases[name]['function']
            mode = ScriptServer._specialCases[name]['mode']
            
        return self.runFunction(func, 
                                {
                                   'component_name':name, 
                                   'parameter_name':value, 
                                   'mode':mode, 
                                   'blocking': blocking
                                })

class ActionLib(object):
    """ Concrete implementation of the robot interface using the ActionLib client """
    
    _specialCases = {
                    'light': {'function': 'set_light', 'mode': ''},
                    'sound': {'function': 'say', 'mode': 'FEST_EN' },
                    'play': {'function': 'play', 'mode': '' }
                    }
    
    def __init__(self):
        self._ros = rosHelper.ROS()
        self._ros.configureROS(packageName='actionlib_interface')
        import actionlib, cob_script_server.msg
        self._ssMsgs = cob_script_server.msg
        
        self._ros.initROS()
        self._doneState = actionlib.SimpleGoalState.DONE
        self._client = actionlib.SimpleActionClient('/script_server', self._ssMsgs.ScriptAction)
        print "Waiting for script_server"
        self._client.wait_for_server()
        print "Connected to script_server"
        
    def runFunction(self, funcName, kwargs):
        func = str(funcName).encode('ascii', 'ignore')
        name = str(kwargs.get('component_name', None)).encode('ascii', 'ignore')
        value = str(kwargs.get('parameter_name', None)).encode('ascii', 'ignore')
        mode = str(kwargs.get('mode', None)).encode('ascii', 'ignore')
        blocking = bool(kwargs.get('blocking', True))
        service_name = str(kwargs.get('service_name', None)).encode('ascii', 'ignore')
        if kwargs.has_key('duration'):
            duration = float(kwargs.get('duration'))
        else:
            duration = None

        goal = self._ssMsgs.ScriptGoal(
                                          function_name=func,
                                          component_name=name,
                                          parameter_name=value,
                                          mode=mode,
                                          service_name=service_name,
                                          duration=duration
                                      )
        
        if blocking:
            self._client.send_goal(goal)
            while self._client.simple_state != self._doneState:
                time.sleep(0.1)
            
            return self._client.get_state()
        else:
            self._client.send_goal(goal)
            return 1
        
    def stopComponent(self, name):
        if name in ActionLib._specialCases:
            return 3

        return self.runFunction("stop", { 'component_name':name, 'blocking':False })
    
    def initComponent(self, name):
        if name not in ActionLib._specialCases.keys():
            return self.runFunction('init', {'component_name': name} )
        return 3
    
    def runComponent(self, name, value, mode=None, blocking=True):
        if name not in ActionLib._specialCases.keys():
            func = "move"
        else:
            func = ActionLib._specialCases[name]['function']
            mode = ActionLib._specialCases[name]['mode']
            
        return self.runFunction(func, 
                                {
                                   'component_name':name, 
                                   'parameter_name':value, 
                                   'mode':mode, 
                                   'blocking': blocking
                                })

class PoseUpdater(robot.PoseUpdater):
    """ Concrete implementation of pose updates for Care-O-Bot robots """
    
    def __init__(self, robot):
        super(PoseUpdater, self).__init__(robot)
        self._rangeSensors = robot_config[robot.name]['phidgets']['topics']
        self._rangeThreshold = robot_config[robot.name]['tray']['size'] / 100.0
        self._rangeWindow = robot_config[robot.name]['phidgets']['windowSize']
        self._rangeHistory = {}
        self._rs = None
        
    @property
    def _ros(self):
        if self._rs == None:
            # Wait to configure/initROS ROS till it's actually needed
            self._rs = rosHelper.ROS()
        return self._rs
    
    def checkUpdatePose(self, robot):
        states = {}
        states.update(self.getTrayStates(robot))
        states.update(self.getHeadStates(robot))
        self.updateStates(states)
        
    def getHeadStates(self, robot):
        """ Returns a dictionary with values for sensor 'eyePosition' to the current angle of the camera """
        return {
                   'eyePosition': self.getComponentPosition(robot, 'head'),
                   }
    
    def getPhidgetState(self):
        """ Return a tuple containing the averaged phidget sensors and a boolean indicating if the tray is empty """
        """ (0.0, True) """
        trayIsEmpty = None                
        
        averages = []
        for topic in self._rangeSensors:
            if not self._rangeHistory.has_key(topic):
                self._rangeHistory[topic] = deque()
            
            rangeMsg = self._ros.getSingleMessage(topic=topic, timeout=0.25)
            if rangeMsg == None:
                if topic not in self._warned: 
                    self._warned.append(topic)
                    print >> sys.stderr, "Phidget sensor not ready before timeout for topic: %s" % topic                
                self._rangeHistory.pop(topic)
                continue
            else:
                if topic in self._warned:
                    self._warned.remove(topic)

            if rangeMsg.range < 0:
                if topic + '_rangeErr' not in self._warned:
                    self._warned.append(topic + '_rangeErr')
                    print >> sys.stderr,  "Phidget sensor returned invalid range! %s:%s" % (topic, rangeMsg.range)
                self._rangeHistory.pop(topic)
                continue
            elif topic + '_rangeErr' in self._warned:
                self._warned.remove(topic + '_rangeErr')

            self._rangeHistory[topic].append(rangeMsg.range)
            if len(self._rangeHistory[topic]) > self._rangeWindow:
                self._rangeHistory[topic].popleft()
            
            averages.append(sum(self._rangeHistory[topic]) / len(self._rangeHistory[topic]))
        
        if len(averages) < len(self._rangeSensors) * 0.60:
            print "Less than 60% of range sensors are functional, tray Full/Empty not available"
            return (None, None)

        if any(map(lambda x: x <= self._rangeThreshold, averages)):
            trayIsEmpty = 'Full'
        else:
            trayIsEmpty = 'Empty'

        p = []
        for position in averages:
            p.append(round(min(position, self._rangeThreshold * 1.25), 3))
   
        return (p, trayIsEmpty)

    def getComponentPosition(self, robot, componentName):
        (stateName, state) = robot.getComponentState(componentName)
        if stateName == None or len(state['positions']) == 0:
            if len(state['positions']) == 0:
                #Error while retreiving state
                return (None, None)

        if stateName == '':
            #print "No named component state for: %s." % (componentName)
            stateName = 'Unknown'
        
        p = []
        for position in state['positions']:
            p.append(round(position, 3))
        
        return (p, stateName)

    def getTrayStates(self, robot):
        """ Returns a dictionary containing the values for sensor variables 'trayStatus' and 'trayIs' """
        return {
                   'trayStatus': self.getComponentPosition(robot, 'tray'),
                   'trayIs': self.getPhidgetState() }
            
if __name__ == '__main__':
    """ Run pose and location updates for the currentRobot """
    from robotFactory import Factory
    robot = Factory.getCurrentRobot()
#     frequency=math.pi*2/100
#     phase1=2
#     phase2=0
#     phase3=4
#     center=128
#     width=127
#     l=50
#     robot = CareOBot('Care-O-Bot 3.2', 'http://cob3-2-pc1:11311')
#     while True:
#         for i in range(0, l):
#             red = (math.sin(frequency*i + phase1) * width + center) / 255
#             grn = (math.sin(frequency*i + phase2) * width + center) / 255
#             blu = (math.sin(frequency*i + phase3) * width + center) / 255
#             robot.setLight([red, grn, blu])

    import locations
    from history import SensorLog
    l = locations.RobotLocationProcessor(robot)
    rp = PoseUpdater(robot)
    sr = SensorLog(rp.channels, rp.robot.name)

    rp.start()
    sr.start()
    
    l.start()
    
    while True:
        try:
            sys.stdin.read()
        except KeyboardInterrupt:
            break
    l.stop()

    sr.stop()
    rp.stop()
