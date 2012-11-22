import io, math, time, sys
from PIL import Image
from extensions import PollingProcessor
from Data.dataAccess import DataAccess
import rosHelper

class PoseUpdater(PollingProcessor):
    def __init__(self, robot=None):
        super(PoseUpdater, self).__init__()
        if robot == None:
            robot = CareOBot()
        self._robot = robot
        self._dao = DataAccess()
        self._ros = rosHelper.ROS()
        self._lastState = { 'raised': None, 'lowered': None, 'empty': None }
    
    def start(self):
        print "Started polling pose for %s" % (self._robot.name)
        self._addPollingProcessor('pose ' + self._robot.name, self.checkUpdatePose, (self._robot, ), 2)
    
    def stop(self):        
        print "Stopped polling location for %s" % (self._robot.name)
        self._removePollingProcessor('pose ' + self._robot.name)
    
    def checkUpdatePose(self, robot):
        self.updateTray(robot)

    def updateTray(self, robot):
        (name, _) = robot.getComponentState('tray')
        trayIsRaised = False
        trayIsLowered = False
        trayIsEmpty = True
        if name == 'up':
            trayIsRaised = True
        elif name == 'down':
            trayIsLowered = True
            
        range0 = self._ros.getSingleMessage(topic='/range_0', timeout=0.25)
        range1 = self._ros.getSingleMessage(topic='/range_0', timeout=0.25)
        range2 = self._ros.getSingleMessage(topic='/range_0', timeout=0.25)
        range3 = self._ros.getSingleMessage(topic='/range_0', timeout=0.25)
        if range0 != None and range1 != None and range2 != None and range3 != None:
            threshold = 0.2
            if range0.range < threshold or range1.range < threshold or range2.range < threshold or range3.range < threshold:
                trayIsEmpty = False
        else:
            print "Phidget sensors not ready before timeout"
        
        update = False
        if self._lastState['lowered'] != trayIsLowered:
            sql = "UPDATE `ActionGoals` SET `value` = %s WHERE `name`='trayIsLowered'"
            args = (trayIsLowered)
            update = True
            self._dao.saveData(sql, args)
        
        if self._lastState['raised'] != trayIsRaised:
            sql = "UPDATE `ActionGoals` SET `value` = %s WHERE `name`='trayIsRaised'"
            args = (trayIsRaised)
            update = True
            self._dao.saveData(sql, args)
        
        if self._lastState['empty'] != trayIsEmpty:
            sql = "UPDATE `ActionGoals` SET `value` = %s WHERE `name`='trayIsEmpty'"
            args = (trayIsEmpty)
            update = True
            self._dao.saveData(sql, args)
            
        self._lastState['raised'] = trayIsRaised
        self._lastState['lowered'] = trayIsLowered
        self._lastState['empty'] = trayIsEmpty
                
        if update:
            print "Updated tray state to Lowered: %(lowered)s, Raised: %(raised)s, Empty: %(empty)s" % { 'lowered': trayIsLowered, 'raised': trayIsRaised, 'empty': trayIsEmpty }

class CareOBot(object):
    _imageFormats = ['BMP', 'EPS', 'GIF', 'IM', 'JPEG', 'PCD', 'PCX', 'PDF', 'PNG', 'PPM', 'TIFF', 'XBM', 'XPM']
    _states = {
            0: 'PENDING',
            1: 'ACTIVE',
            2: 'PREEMPTED',
            3: 'SUCCEEDED',
            4: 'ABORTED',
            5: 'REJECTED',
            6: 'PREEMPTING',
            7: 'RECALLING',
            8: 'RECALLED',
            9: 'LOST'}

    def __init__(self, name='Care-O-Bot 3.2'):
        self._rs = None
        self._ss = None
        self._poseProcessor = None
        self._name = name
        
    @property
    def name(self):
        return self._name
    
    @property
    def _pose(self):
        if self._poseProcessor == None:
            self._poseProcessor = Pose()
        return self._poseProcessor
    
    @property
    def _ros(self):
        if self._rs == None:
            #Wait to configure/initROS ROS till it's actually needed
            self._rs = rosHelper.ROS()
        return self._rs
            
    @property
    def _cob(self):
        if self._ss == None:
            #Wait to configure/initROS ROS till it's actually needed
            self._ss = ActionLib()
            #self._ss = ScriptServer()
        return self._ss
    
    def getImage(self, leftRight='right', retFormat='PNG'):
        topic = '/stereo/%(camera)s/image_color/compressed' % { 'camera' : leftRight }

        img_msg = self._ros.getSingleMessage(topic)
        if img_msg == None:
            return None
        
        imgBytes = io.BytesIO()
        imgBytes.write(img_msg.data)
        
        angle = self.getCameraAngle()

        imgBytes.seek(0)
        img = Image.open(imgBytes)
            
        #0=back, 180=front, 270=top, 90=bottom.  rotate if not front (0-180 are invalid angles, only included for 'buffer')
        if angle >= 90 and angle <= 270:
            pass
        else:
            img = img.rotate(180)
        
        retFormat = retFormat.upper()
        if retFormat == 'JPG':
            retFormat = 'JPEG'
            
        if retFormat not in CareOBot._imageFormats:
            retFormat = 'PNG' 
        
        imgBytes.seek(0)
        img.save(imgBytes, retFormat)

        return imgBytes.getvalue()

    def getCameraAngle(self):
        cameraState = self.getComponentState('head', True)
        
        angle = round(math.degrees(cameraState.actual.positions[0]), 2)
        angle = angle % 360
            
        return angle
        
    def executeFunction(self, funcName, kwargs):
        return self._cob.runFunction(funcName, kwargs)
    
    def getLocation(self, raw=False):
        p = self._pose
        ((x, y, _), rxy) = p.getRobotPose()
        if x == None or y == None:
            if raw:
                return (None, None, None)
            else:
                return (None, (None, None, None))
        
        angle = round(math.degrees(rxy))
        pos = (round(x, 3), round(y, 3), angle)
        
        if raw:
            return pos
        else:
            return self.resolveLocation(pos)

    def resolveLocation(self, curPos, maxDistance=None):
        dao = DataAccess()

        try:
            locations = dao.findLocations()
        except:
            return ('', curPos)

        if len(locations) == 0:
            return ('', curPos)

        name = None
        diff = None
        for loc in locations:
            dist = 0
            dist += math.pow(curPos[0] - loc['xCoord'], 2)
            dist += math.pow(curPos[1] - loc['yCoord'], 2)
            #We're not worried about orientation for location matching
            #use a fraction of the orientation in case we get two named
            # locations with the same x/y and different orientations
            # needs to be a very small fraction since rotation uses a different unit measurement
            # than location does (degrees vs. meters)
            dist += math.pow(curPos[2] - loc['orientation'], 2) / 100000
            dist = math.sqrt(dist)
            if name == None or dist < diff:
                name = loc['name']
                diff = dist

        if maxDistance == None or diff <= maxDistance:
            return (name, curPos)
        else:
            return ('', curPos)

    def setLight(self, colour):
        self._cob.runFunction('set_light', {'parameter_name': colour})

    def setComponentState(self, name, value):
        #check if the component has been initialised, and init if it hasn't
        if len(self._ros.getTopics('/%(name)s_controller' % { 'name': name })) == 0:
            self._cob.initComponent(name)
        status = self._cob.runComponent(name, value)
        #There is a bug in the Gazebo COB interface that prevents proper trajectory tracking
        #this causes most status messages to come back as aborted while the operation is still
        #commencing, time delay to attempt to compensate...
        if status != 3 and len(self._ros.getTopics('/gazebo')) > 0:
            time.sleep(5)
            print >> sys.stderr, 'Gazebo hack: state ' + CareOBot._states[status] + ' changed to state ' + CareOBot._states[3]
            return CareOBot._states[3]
        
        return CareOBot._states[status]
        
    def getComponentPositions(self, componentName):
        try:
            self._ros.configureROS(packageName='rospy')
            import rospy
            return rospy.get_param('script_server/%s' % (componentName))
        except:
            return []

    def getComponents(self):
        try:
            self._ros.configureROS(packageName='rospy')
            import rospy
            return rospy.get_param('script_server').keys()
        except:
            return []
        
    def getComponentState(self, componentName, raw=False):
        topic = '/%(name)s_controller/state' % { 'name': componentName }
        state = self._ros.getSingleMessage(topic)
                
        if raw:
            return state
        else:
            return self.resolveComponentState(componentName, state)
    
    def resolveComponentState(self, componentName, state, tolerance=0.10):
        curPos = state.actual.positions

        positions = self.getComponentPositions(componentName)

        if len(positions) == 0:
            return ('', curPos)

        name = None
        diff = None
        for positionName in positions:
            positionValue = self.getValue(positions[positionName])
            if type(positionValue) is not list:
                #we don't currently handle nested types
                continue

            if len(positionValue) != len(curPos):
                #raise Exception("Arguement lengths don't match")
                continue
            
            dist = 0
            for index in range(len(positionValue)):
                dist += math.pow(curPos[index] - positionValue[index], 2)
            dist = math.sqrt(dist)
            if name == None or dist < diff:
                name = positionName
                diff = dist
                        
        if diff <= tolerance:
            return (name, curPos)
        else:
            return ('', curPos)
    
    def getValue(self, val):
        if type(val) is list:
            ret = val[0]
        else:
            ret = val
        
        return ret

class ScriptServer(object):

    def __init__(self):
        self._ros = rosHelper.ROS()
        self._ros.configureROS(packageName='simple_script_server')
        import simple_script_server
        self._ros.initROS()
        self._ss = simple_script_server.simple_script_server()
    
    def runFunction(self, funcName, kwargs):
        try:
            func = getattr(self._ss, funcName)
        except AttributeError:
            raise Exception('Unknown function: %s' % (funcName))
        
        return func(**kwargs)
        
    
    def initComponent(self, name):
        return self._ss.initROS(name, True).get_state()
    
    def runComponent(self, name, value, mode='', blocking=True):
        if name == 'light':
            return self._ss.set_light(value, blocking).get_state()
        elif name == 'sound':
            return self._ss.say(value, blocking).get_state()
        else:
            return self._ss.move(name, value, blocking, mode).get_state()

class Pose(object):
    def __init__(self):
        ros = rosHelper.ROS()
        ros.configureROS(packageName='cob_map_pose')
        import tf, rospy
        self._rospy = rospy
        self._tf = tf
        ros.initROS()
        self._listener = self._tf.TransformListener()
    
    def getRobotPose(self):
        """
        Waits for the /base_footprint to /map transform to be availalble and 
        returns two tuples: (x, y, z) and a quaternion ( rx, ry, rz, rxy)
        Note: z values are 0 for 2D mapping and navigation.
        """
        # Wait for tf to get the frames
        now = self._rospy.Time.now()
        try:
            self._listener.waitForTransform("/map", "/base_footprint", now, self._rospy.Duration(1.0))
        except self._tf.Exception as e:
            if str(e) != 'Unable to lookup transform, cache is empty, when looking up transform from frame [/base_footprint] to frame [/map]':
                print >> sys.stderr, "Error while waiting for transform: " + str(e)
            return ((None, None, None), None)
        
        try:
            (xyPos, heading) = self._listener.lookupTransform('/map', '/base_footprint', now)
            (_, _, orientation) = self._tf.transformations.euler_from_quaternion(heading)
            return (xyPos, orientation)
        except (self._tf.LookupException, self._tf.ConnectivityException, self._tf.ExtrapolationException) as e:
            print >> sys.stderr, "Error while looking up transform: " + str(e)
            return ((None, None, None), None)

class ActionLib(object):
    _specialCases = {
                    'light': {'function': 'set_light', 'mode': ''},
                    'sound': {'function': 'say', 'mode': 'FEST_EN' }
                    }
    
    def __init__(self):
        self._ros = rosHelper.ROS()
        self._ros.configureROS(packageName='actionlib_interface')
        import actionlib, cob_script_server.msg
        self._ssMsgs = cob_script_server.msg
        
        self._ros.initROS()
        self._client = actionlib.SimpleActionClient('/script_server', self._ssMsgs.ScriptAction)
        self._client.wait_for_server()
        
    def runFunction(self, funcName, kwargs):
        name = None
        value = None
        mode = None
        blocking = None
        service_name = None
        duration = None
        
        if kwargs.has_key('component_name'):
            name = str(kwargs['component_name']).encode('ascii', 'ignore')
            
        if kwargs.has_key('parameter_name'):
            value = str(kwargs['parameter_name']).encode('ascii', 'ignore')
                        
        if kwargs.has_key('mode'):
            mode = str(kwargs['mode']).encode('ascii', 'ignore')
            
        if kwargs.has_key('blocking'):
            blocking = bool(kwargs['blocking'])
            
        if kwargs.has_key('service_name'):
            service_name = str(kwargs['service_name']).encode('ascii', 'ignore')
            
        if kwargs.has_key('duration'):
            duration = float(kwargs['duration'])

        goal = self._ssMsgs.ScriptGoal(
                          function_name=str(funcName).encode('ascii', 'ignore'),
                          component_name=name,
                          parameter_name=value,
                          mode=mode,
                          blocking=blocking,
                          service_name=service_name,
                          duration=duration
                          )
        
        return self._client.send_goal_and_wait(goal)
        
    def initComponent(self, name):
        if name not in ActionLib._specialCases.keys():
            func = 'init'
            goal = self._ssMsgs.ScriptGoal(
                              function_name=func.encode('ascii', 'ignore'),
                              component_name=name.encode('ascii', 'ignore'),
                              blocking=True)
            return self._client.send_goal_and_wait(goal)
        return 3
    
    def runComponent(self, name, value, mode=None, blocking=True):
        if name not in ActionLib._specialCases.keys():
            func = "move"
        else:
            func = ActionLib._specialCases[name]['function']
            mode = ActionLib._specialCases[name]['mode']

        goal = self._ssMsgs.ScriptGoal(
                          function_name=str(func).encode('ascii', 'ignore'),
                          component_name=str(name).encode('ascii', 'ignore'),
                          parameter_name=str(value).encode('ascii', 'ignore'),
                          mode=str(mode).encode('ascii', 'ignore'),
                          blocking=bool(blocking)
                          )
        
        status = self._client.send_goal_and_wait(goal)
        return status

if __name__ == '__main__':
    #c = CareOBot()
    #l = c.executeFunction('say', {'parameter_name': ['I can talk!'], 'blocking': True})
    #print l
    #l = c.setComponentState('tray', 'down')
    
    #rosHelper.ROS.configureROS(packageName='cob_script_server', packagePath='/home/nathan/git', rosMaster='http://cob3-2-pc1:11311')
    #a = ActionLib()
    #print 'Init: ' + str(a.initComponent('torso'))
    #print 'Reco: ' + str(a.runFunction('recover', {'component_name': 'torso', 'blocking':True}))
    #print 'Move: ' + str(a.runComponent('torso', 'front', True))
    #print 'Move: ' + str(a.runComponent('torso', 'home', True))
    
    rosHelper.ROS.configureROS(packageName='cob_script_server', packagePath='/home/nathan/git', rosMaster='http://cob3-2-pc1:11311')
    p = PoseUpdater()
    p.start()
    
    while True:
        try:
            sys.stdin.read()
        except KeyboardInterrupt:
            break
        
    p.stop()
