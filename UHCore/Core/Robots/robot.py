# Add project reference
import sys, os
path = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../')
sys.path.append(path)

import io, math, time
from extensions import PollingProcessor
from Data.dataAccess import Sensors, Users
from Data.proxemics import ProxemicMover
from config import robot_config
import Data.dataAccess

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

class PoseUpdater(PollingProcessor):
    """Abstract processor used to update the sensors table for sensors (real and meta) located on the robot"""
    """checkUpdatePose() must be implemented by the inheriting class""" 
    
    def __init__(self, robot):
        super(PoseUpdater, self).__init__()
        self._robot = robot
        self._sensors = Sensors().findSensors(None, False)
        self._channels = {}
        self._warned = []
    
    def start(self):
        print "Started polling pose for %s" % (self._robot.name)
        self._addPollingProcessor('pose ' + self._robot.name, self.checkUpdatePose, (self._robot,), .25)
    
    def stop(self):        
        print "Stopped polling pose for %s" % (self._robot.name)
        self._removePollingProcessor('pose ' + self._robot.name)
    
    def updateStates(self, states):
        """ Iterates the states list and adds the values to the _channels dict in the standard format """
        """ See PollingProcessor for _channels details """
        for key, value in states.items():
            if value != None and value[1] != None:
                try:
                    sensor = next(s for s in self._sensors if s['name'] == "%s" % (key))
                    if key in self._warned:
                        self._warned.remove(key)
                except StopIteration:
                    if key not in self._warned:
                        print >> sys.stderr, "Warning: Unable to locate sensor record for %s sensor %s." % (self._robot.name, key)
                        self._warned.append(key)
                    continue
                
                _id = sensor['sensorId']
                self._channels[key] = {
                                         'id': _id,
                                         'room': self._robot.name,
                                         'channel': key,
                                         'value': value[0],
                                         'status': value[1] }
        
    @property
    def robot(self):
        return self._robot
    
    @property
    def channels(self):
        if self._channels == None:
            self._channels = {}
        
        return self._channels
    
    def checkUpdatePose(self, robot):
        pass

class Robot(object):
    """Abstract base class for all robots implemented in this module"""
    """Contains code that has thus far proven generic to all used robots"""
     
    _imageFormats = ['BMP', 'EPS', 'GIF', 'IM', 'JPEG', 'PCD', 'PCX', 'PDF', 'PNG', 'PPM', 'TIFF', 'XBM', 'XPM']

    def __init__(self, name, robotInterface):
        self._name = name
        self._robIntClass = robotInterface
        self._robIntInstance = None
        
    @property
    def _robInt(self):
        """Concrete implementation of the robot interface"""
        if self._robIntInstance == None:
            self._robIntInstance = self._robIntClass()
            
        return self._robIntInstance
        
    @property
    def name(self):
        return self._name
                
    def getImage(self, retFormat='PNG'):
        """ Abstract, should be implemented by robots supporting image processing""" 
        return None
    
    def stop(self, name=None):
        """ Attempt to stop the named component, or all components if no name supplied, blocks until complete"""
        if name==None or name == "":
            for component in self.getComponents():
                self._robInt.stopComponent(component)
        else:
            self._robInt.stopComponent(name)
    
    def play(self, fileName, blocking=True):
        """ Abstract, Play a specified file. File should be assumed to be accessable to the robot. """ 
        print "Play: %s" % fileName 
    
    def say(self, text, languageCode="en-gb", blocking=True):
        """ Abstract, Use the TTS engine to say the specified text, using the specified language code. """
        """ Language code should be assumed to be in iso 639-1 format, with option region tag, defaults to UK English """
        print "Say (%s): %s" % (languageCode, text)
        
    def sleep(self, milliseconds):
        """ Pause execution for specified time(in milliseconds) """
        time.sleep(milliseconds / 1000.0)

    def executeFunction(self, funcName, kwargs):
        """ Attempt to run an arbitrary function on the robot interface """
        return self._robInt.runFunction(funcName, kwargs)
    
    def getLocation(self, dontResolveName=False):
        """ Abstract, Returns a tuple containing ('LocName or Empty', (X, Y, Theta))"""
        return ('', (None, None, None))

    def setLight(self, colour):
        """ Set the light to the named colour or [R,G,B] """
        self._robInt.runComponent('light', colour)

    def setComponentState(self, name, value, blocking=True):
        """
            Set the named component to the given value
            value can be a string for a named position, or an array of floats for specific joint values
                The length of the array must match the length of the named joints for the component
                If a named position is defined in the robot config, the config value will be substituted 
                 before sending.  This allows for a generic top level command to work on multiple robots.
                 I.E. 'Tray' to 'Raised' will change to 'Up', 'Open', 'DeliverUp' depending on the model
                 of robot (COB3.2, Sunflower, COB3.5, in this example)
            if name is base and value is userLocation, this will cause the robot to go to the current
                user location.  
            set blocking to true(default) for this function to block until completed
            if blocking is set to false, this function will return immediately with value 'ACTIVE'
        """
        
        if type(value) == str and \
           robot_config.has_key(self.name) and \
           robot_config[self.name].has_key(name) and \
           robot_config[self.name][name].has_key('positions') and \
           robot_config[self.name][name]['positions'].has_key(value):
            value = robot_config[self.name][name]['positions'][value]

        if name == "base" and value == "userLocation":
            user = Users().getActiveUser()
            if user['xCoord'] != None and user['yCoord'] != None and user['orientation'] != None:
                value = [user['xCoord'], user['yCoord'], math.radians(user['orientation'])]
                print "Going to userLocation location (%s, %s, %s)" % (
                                                                        user['xCoord'], 
                                                                        user['yCoord'], 
                                                                        user['orientation'])

        status = self._robInt.runComponent(name, value, None, blocking)
        return _states[status]
    
    def getComponentPositions(self, componentName):
        """ Abstract, Returns a dictionary that contains all available named positions and their """
        """     associated value arrays {'PositionName', [values, ...]} """
        return {}

    def getComponents(self):
        """ Abstract, Returns a list of all available components """
        return []
        
    def getComponentState(self, componentName, dontResolveName=False):
        """ Abstract, return the a tuple containing the state (named and raw) of the named component.  """
        """ Name resolution can be skipped by setting dontResolveName=True. """
        """ ('Name or Empty', {'name': '...', 'positions': [..., ...], 'goals': [..., ...], 'joints': [..., ...] })"""
        ret = {'name': '', 'positions': [], 'goals': [], 'joints': [] }
        return ('', ret)
        
    def resolveComponentState(self, componentName, state, tolerance=0.5):
        """ Attempt to resolve a given state to it's closes named position, given the stated tolerance value"""
        """ This method was initally written for locations, and can struggle with higher dimensional values, """
        """     I.E, a named arm position.  There are also problems around '0'.  This method will likely need to """
        """     eventually be replaced with something more robust. """
        """ If the named position exists in the robot_config, the config key will be substituted before returning """
        """  This is the reverse of what happens in setComponentState, using the same example, ('Up', 'Open', 'DeliverUp') """
        """  will all be changed to 'Raised' before returning """
        if state == None:
            return (None, None)
        
        curPos = state['positions']

        positions = self.getComponentPositions(componentName)
        
        if len(positions) == 0:
            #print >> sys.stderr, "Unable to retrieve named positions. Name resolution will now abort"
            return (None, state)

        name = None
        diff = None
        for positionName in positions:
            positionValue = self._getValue(positions[positionName])
            if type(positionValue) is not list:
                # we don't currently handle nested types
                continue

            if len(positionValue) != len(curPos):
                # raise Exception("Argument lengths don't match")
                continue
            
            dist = 0
            for index in range(len(positionValue)):
                dist += math.pow(curPos[index] - positionValue[index], 2)
            dist = math.sqrt(dist)
            if name == None or dist < diff:
                name = positionName
                diff = dist
                        
        if diff <= tolerance:
            if robot_config.has_key(self.name) and robot_config[self.name].has_key(componentName) and robot_config[self.name][componentName].has_key('positions'):
                positions = robot_config[self.name][componentName]['positions']
                for key, value in positions.items():
                    if value == name:
                        return (key, state)
            return (name, state)
        else:
            return ('', state)
    
    def _getValue(self, val):
        if type(val) is list:
            ret = val[0]
        else:
            ret = val
        
        return ret


class ROSRobot(Robot):
    """ Abastract base class for robots that are based on the ROS architecture """
    _imageFormats = ['BMP', 'EPS', 'GIF', 'IM', 'JPEG', 'PCD', 'PCX', 'PDF', 'PNG', 'PPM', 'TIFF', 'XBM', 'XPM']

    def __init__(self, name, robotInterface, serverTopic, imageTopic):
        """ Assumes a service structure similar to that of the IPA Care-O-Bot """
        """ Robots known to work are the IPA Care-O-Bot (3.2,3.5,3.6) and the UH Sunflower (1.1, 1.2) """
        super(ROSRobot, self).__init__(name, robotInterface)
        self._rs = None
        self._tf = None
        self._serverTopic = serverTopic
        self._imageTopic = imageTopic
    
    @property
    def _transform(self):
        """ Transform between the robot base frame and the map frame, used in getLocation() """
        if self._tf == None:
            try:
                import rosHelper
                self._tf = rosHelper.Transform(rosHelper=self._rs, toTopic='/base_footprint', fromTopic='/map')
            except Exception as e:
                print >> sys.stderr, "Error occured while calling transform: %s" % repr(e)
        return self._tf
        
    @property
    def _ros(self):
        """ ROS Interface, current options are rosHelper(stable) and rosMulti(incomplete) """
        # Wait to configure/initROS ROS till it's actually needed
        if self._rs == None:
            import rosHelper
            self._rs = rosHelper.ROS()
        return self._rs
    
    def getImage(self, retFormat='PNG'):
        """ Returns the image from the robots camera (topic specified in robot_config) or None if no camera """
        """ While retFormat is taken into account, PIL appers to always return a JPG file, issue has not been investigated """
        if not robot_config.has_key(self.name) or not robot_config[self.name]['head'].has_key('camera'):
            return None
        
        img_msg = self._ros.getSingleMessage(self._imageTopic)
        if img_msg == None:
            return None
        
        from PIL import Image
        imgBytes = io.BytesIO()
        imgBytes.write(img_msg.data)
        
        imgBytes.seek(0)
        img = Image.open(imgBytes)

        if robot_config[self._name]['head']['camera'].has_key('rotate'):
            angle = self.getCameraAngle() or 0
    
            a = abs(angle - robot_config[self._name]['head']['camera']['rotate']['angle'])
            if a > 180:
                a = abs(a - 360)
            
            # 0=back, 180=front, 270=top, 90=bottom.  rotate if not front (0-180 are invalid angles, only included for 'buffer')
            # if angle <= 90 and angle >= 270:
            if a <= robot_config[self._name]['head']['camera']['rotate']['distance']:
                img = img.rotate(robot_config[self._name]['head']['camera']['rotate']['amount'])
        
        retFormat = retFormat.upper()
        if retFormat == 'JPG':
            retFormat = 'JPEG'
            
        if retFormat not in Robot._imageFormats:
            retFormat = 'PNG' 
        
        imgBytes.seek(0)
        img.save(imgBytes, retFormat)

        return imgBytes.getvalue()

    def getImageOverhead(self, retFormat='PNG'):
        """ Returns the image from the overhead camera (topic is '/camera_living/image_raw/compressed' ) """
        """ While retFormat is taken into account, PIL appers to always return a JPG file, issue has not been investigated """
        #if not robot_config.has_key(self.name) or not robot_config[self.name]['head'].has_key('camera'):
        #    return None
        
        img_msg = self._ros.getSingleMessage('/camera_living/image_raw/compressed')
        if img_msg == None:
            return None
        
        from PIL import Image
        imgBytes = io.BytesIO()
        imgBytes.write(img_msg.data)
        
        #imgBytes.seek(0)
        #img = Image.open(imgBytes)

        #if robot_config[self._name]['head']['camera'].has_key('rotate'):
        #    angle = self.getCameraAngle() or 0
    
        #    a = abs(angle - robot_config[self._name]['head']['camera']['rotate']['angle'])
        #    if a > 180:
        #        a = abs(a - 360)
            
            # 0=back, 180=front, 270=top, 90=bottom.  rotate if not front (0-180 are invalid angles, only included for 'buffer')
            # if angle <= 90 and angle >= 270:
        #    if a <= robot_config[self._name]['head']['camera']['rotate']['distance']:
        #        img = img.rotate(robot_config[self._name]['head']['camera']['rotate']['amount'])
        
        #retFormat = retFormat.upper()
        #if retFormat == 'JPG':
        #    retFormat = 'JPEG'
            
        #if retFormat not in Robot._imageFormats:
        #    retFormat = 'PNG' 
        
        #imgBytes.seek(0)
        #img.save(imgBytes, retFormat)

        return imgBytes.getvalue()
    
    def getLocation(self, dontResolveName=False):
        """ Returns a tuple containing ('LocName or Empty', (X, Y, Theta))"""
        tf = self._transform
        if tf == None:
            return ('', (None, None, None))

        ((x, y, _), rxy) = tf.getTransform()
        if x == None or y == None:
            return ('', (None, None, None))
        
        angle = round(math.degrees(rxy))
        pos = (round(x, 3), round(y, 3), angle)
        
        if dontResolveName:
            return ('', pos)
        else:
            return Data.dataAccess.Locations.resolveLocation(pos)

    def setComponentState(self, name, value, blocking=True):
        """
            Set the named component to the given value
            value can be a string for a named position, or an array of floats for specific joint values
                The length of the array must match the length of the named joints for the component
            if name is base and value is userLocation, this will cause the proxemics module to be called
                to send the robot near the users current location
            set blocking to true(default) for this function to block until completed
            if blocking is set to false, this function will return immediately with value 'ACTIVE'
        """
        if name == "base" and value == "userLocation":
            user = Users().getActiveUser()
            if user['xCoord'] != None and user['yCoord'] != None and user['orientation'] != None:
                try:
                    p = ProxemicMover(self)
                    if p.gotoTarget(user['userId'], user['poseId'], user['xCoord'], user['yCoord'], user['orientation']):
                        return _states[3]
                    else:
                        pass
                        #return _states[4]
                except Exception as e:
                    print >> sys.stderr, "Exception occurred while calling proxemics: %s" % e
                
                value = [user['xCoord'], user['yCoord'], math.radians(user['orientation'])]
                print >> sys.stderr, "Proxemics failed, proceeding directly to location (%s, %s, %s)" % (
                                                                                                            user['xCoord'], 
                                                                                                            user['yCoord'], 
                                                                                                            user['orientation'])
            else:
                print >> sys.stderr, "Could not go to user location, missing information.  X:%s, Y:%s, T:%s" % (
                                                                                                            user['xCoord'], 
                                                                                                            user['yCoord'], 
                                                                                                            user['orientation'])
        
        status = super(ROSRobot, self).setComponentState(name, value, blocking)
        # There is a bug in the Gazebo COB interface that prevents proper trajectory tracking
        # this causes most status messages to come back as aborted while the operation is still
        # commencing, time delay to attempt to compensate...
        if status != _states[3] and len(self._ros.getTopics('/gazebo')) > 0:
            time.sleep(1)
            print >> sys.stderr, 'Gazebo hack: state ' + status + ' changed to state ' + _states[3]
            status = _states[3]
        
        return status

    def getComponentPositions(self, componentName):
        """ Abstract, Returns a dictionary that contains all available named positions and their """
        """     associated value arrays {'PositionName', [values, ...]} """
        return self._ros.getParam('%s/%s' % (self._serverTopic, componentName))

    def getComponents(self):
        """ Returns a list of all available components """
        return self._ros.getParam(self._serverTopic).keys()
        
    def getComponentState(self, componentName, dontResolveName=False):
        """ Return the a tuple containing the state (named and raw) of the named component.  """
        """ Name resolution can be skipped by setting dontResolveName=True. """
        """ ('Name or Empty', {'name': '...', 'positions': [..., ...], 'goals': [..., ...], 'joints': [..., ...] })"""
        topic = '/%(name)s_controller/state' % { 'name': componentName }
        state = self._ros.getSingleMessage(topic)
        
        try:
            ret = {'name': componentName, 'positions': state.actual.positions, 'goals': state.desired.positions, 'joints': state.joint_names }
        except:
            print "Error retrieving joint state" 
            ret = {'name': componentName, 'positions': (), 'goals': (), 'joints': () }
            
        if dontResolveName:
            return ('', ret)
        else:
            return self.resolveComponentState(componentName, ret)

if __name__ == "__main__":
    from robotFactory import Factory
    c = Factory.getCurrentRobot()
    print c.getComponents()
