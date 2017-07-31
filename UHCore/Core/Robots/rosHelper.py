import time, os, sys
from subprocess import Popen, PIPE
from httplib import CannotSendRequest

try:
    from config import ros_config
except:
    ros_config = {}
    
from threading import RLock

_threadLock = RLock()

class ROS(object):
    """ Interface class to ROS libraries and messaging system """
    _activeVersion = None
    _envVars = {}
    _userVars = None
    
    def __init__(self):
        ROS.configureROS(packageName='rospy')
        import rospy
        self._rospy = rospy
        self._topicTypes = {}
        self._subscribers = {}
        self.initROS()
        
    def __del__(self):
        """ Unregisters all subscribers when class is GC'd"""
        if hasattr(self, '_subscribers'):
            for sub in self._subscribers.values():
                sub.unregister()
        
    def initROS(self, name='rosHelper'):
        """ Activate ros, if necessary, safe for subsequent calls """
        with _threadLock:
            if not self._rospy.core.is_initialized():
                self._rospy.init_node('rosHelper', anonymous=True, disable_signals=True)
        
    def getSingleMessage(self, topic, dataType=None, retryOnFailure=1, timeout=None):
        """ Return the first message recieved on the specified topic """
        """ if dataType is not specified, it will be automatically detected and imported by inspecting the topic """
        """ defaults to one retry after a failure (aids with inconsistent network connections """
        """ Timeout value, specified in seconds, is the time to wait for a message, None==Wait forever """ 
        try:
            if dataType == None:
                if not self._topicTypes.has_key(topic):
                    self._topicTypes[topic] = self.getMessageType(topic)
                    
                dataType = self._topicTypes[topic]

            if not self._subscribers.has_key(topic):
                self._subscribers[topic] = RosSubscriber(topic, dataType)

            subscriber = self._subscribers[topic]
            while not subscriber.hasNewMessage:
                if timeout != None:
                    if timeout < 0:
                        raise Exception("Timeout while waiting on messages")
                    else:
                        timeout -= 0.01
                time.sleep(0.01)                
    
            return subscriber.lastMessage
        except:
            if retryOnFailure > 0:
                return self.getSingleMessage(topic, dataType, retryOnFailure - 1, timeout)
            else:
                return None
    
    def getParam(self, paramName, retry=5):
        """ Return the value of a param on the parameter server, equivalent to rosparam 'paramName' """
        """ Default to retry 5 times on failure, this was added due to seemingly fleeting connection """
        """     problems with the rosparam server """
        with _threadLock:
            try:
                return self._rospy.get_param(paramName)
            except CannotSendRequest:
                if retry > 0:
                    return self.getParam(paramName, retry - 1)
                else:
                    print >> sys.stderr, "Unable to connect to ros parameter server, Timeout sending request"
                    return []                                        
            except Exception as e:
                print >> sys.stderr, "Unable to connect to ros parameter server, Error: %s" % repr(e)
                return []
    
    def getTopics(self, baseFilter='', exactMatch=False, retry=10):
        """ topics = self._rospy.get_published_topics(baseFilter) """
        """ if len(topics) == 0 and baseFilter.strip('/').find('/') == -1: """
        """ """
        """ decided to do filtering a little different than ros """
        """ ros requires an exact match (of the parent namespace) """
        """ this can grab any partial matches """
        """ """
        """ ros doesn't return topics when the full namespace is specified """
        """ i.e. head_controller works and brings back all nested topics """
        """ but head_controller/state does not """
        """ in this case, get all of them and loop through """
        topics = []
        with _threadLock:
            try:
                allTopics = self._rospy.get_published_topics()
            except Exception as e:
                if type(e) != CannotSendRequest:
                    print >> sys.stderr, "Error while retrieving topics, will retry %s more times. Error: %s" % (retry, repr(e))
                if(retry > 0):
                    return self.getTopics(baseFilter, exactMatch, retry - 1)
                else:
                    return topics
        
        if baseFilter.startswith('/'):
            baseFilter = baseFilter[1:]
        for t in allTopics:
            name = t[0]
            if name.startswith('/'):
                name = name[1:]
            if name.strip('/') == baseFilter.strip('/') or (not exactMatch and name.startswith(baseFilter)):
                topics.append(t)
            
        return topics

    def getMessageType(self, topic):
        """ Inspect a topic to discover the message type then import the manifest and the message namespace """
        """ Returns the python type() for the message """
        pubTopic = self.getTopics(topic, True)
        if len(pubTopic) != 0: 
            controller_msgType = pubTopic[0][1]
        else:
            raise Exception('Could not determine ROS messageType for topic: %s' % (topic))
                
        (manifest, cls) = controller_msgType.split('/')
        
        try:
            import roslib
            roslib.load_manifest(manifest)
            
            ns = __import__(manifest + '.msg', globals(), locals(), [cls], -1)
            msgCls = getattr(ns, cls)
            return msgCls
        except Exception as e:
            raise Exception('Error occured while loading message class: %s' % (e))
    
    @staticmethod
    def _getUserVars():
        """ Load variables out of the users bashrc file.  This is added for convenience for users who """
        """ Configure ROS in their bashrc, but can potentially cause lockups as it forces a bash shell to interactive mode """
        """ In order to read the variables."""  
        if ROS._userVars == None:
            #This is a bit more dangerous as it loads the users .bashrc file in a forced interactive shell
            #while not actually being in an interactive shell.  any prompts could cause lockups
            command = ['bash', '-i', '-c', ('%s; env' % ". %s/.bashrc" % os.getenv("HOME")).strip('; ')]
            pipe = Popen(command, stdout=PIPE, stderr=PIPE)
            (data, _) = pipe.communicate()
            env = {}
            for line in data.splitlines():
                try:
                    key, value = line.split("=", 1)
                    env[key] = value
                except ValueError:
                    #TODO: This happens when an environment value has a newline in it
                    #  should fine a proper way to handle it in the future
                    continue
            #env = dict((line.split("=", 1) for line in data.splitlines()))
            ROS._userVars = env
        
        return ROS._userVars
    
    @staticmethod
    def _locateRosOverlayPath():
        env = ROS._getUserVars()
            
        if env.has_key('ROS_PACKAGE_PATH'):
            return env['ROS_PACKAGE_PATH']
        
        return None
    
    @staticmethod
    def _locateRosVersion(): 
        if ROS._activeVersion == None:
            env = ROS._getUserVars()
            if env.has_key('ROS_DISTRO'):
                ROS._activeVersion = env['ROS_DISTRO']
            else:
                bash = open("%s/.bashrc" % os.getenv("HOME"), "r")
                bashlines = bash.readlines()
                # assume the source line is closest to the end
                for n in range(len(bashlines) - 1, 0, -1):
                    # strip lines of white spaces, look for source /opt/ros/.../setup.bash command,
                    # distribution name is between ros/ and next /
                    if bashlines[n].strip().find("source /opt/ros/") == 0:
                        line = bashlines[n].strip()
                        version = line[16:line.find("/", 16)]
                        print "Active ROS version found: ", version
                        ROS._activeVersion = version
                        break
                    
        return ROS._activeVersion
    
    @staticmethod
    def _parseRosVersionSetupBash(version, onlyDifferent=True):
        """ Load the setup.bash file in a bash terminal, and import the env variables into a dict """
        """ onlyDifferent==True(default): Compare this dict to the current env vars and only return """
        """ Variables that are new or different in the ros bash terminal """ 
        if not ROS._envVars.has_key(version):
            # executes the bash script and exports env vars
            bashScript = '/opt/ros/%s/setup.bash' % version
            diffEnv = {}
            if os.path.exists(bashScript):
                rosEnv = ROS._parseBashEnviron('source ' + bashScript)
                baseEnv = ROS._parseBashEnviron()
        
                # find all the variables that ros added/changed
                for key, value in rosEnv.items():
                    if not baseEnv.has_key(key):
                        diffEnv[key] = value
                    elif baseEnv[key] != value:
                        # We really only want the bit that ros added
                        diffEnv[key] = value.replace(baseEnv[key], '').strip(':')
        
                # Add in any overrides from the config file
                if ros_config.has_key('envVars'):
                    diffEnv.update(ros_config['envVars'])
                    rosEnv.update(ros_config['envVars'])
            else:
                print >> sys.stderr, "Unable to read ros bash script, file not found: %s" % bashScript
                
            ROS._envVars[version] = (diffEnv, rosEnv)

        if onlyDifferent:
            return ROS._envVars[version][0]
        else:
            return ROS._envVars[version][1]

    @staticmethod
    def _parseBashEnviron(preCommand=''):
        """ Execute a bash command and export the env to a dictionary """
        command = ['bash', '-c', ('%s; env' % preCommand).strip('; ')]
        pipe = Popen(command, stdout=PIPE)
        data = pipe.communicate()[0]
        env = dict((line.split("=", 1) for line in data.splitlines()))
        return env

    @staticmethod
    def configureROS(version=None, packagePath=None, packageName=None, rosMaster=None, overlayPath=None):
        """ Perform all tasks necessary to configure ROS interface for use.  All variables not specified """
        """     are detected from first the ros_config(if specified) or from the users .bashrc """
        """ Most common use is configureRos(packageName='name') in order to import a ros package for use"""
        """ Can be called multiple times without issue """
        if version == None:
            if not ros_config.has_key('version'):
                version = ROS._locateRosVersion()
            else:
                version = ros_config['version']
        
        if overlayPath == None:
            if not ros_config.has_key('overlayPath'):
                overlayPath = ROS._locateRosOverlayPath()
            else:
                overlayPath = ros_config['overlayPath']

        if(rosMaster == None and ros_config.has_key('rosMaster')):
            rosMaster = ros_config['rosMaster']

        for k, v in ROS._parseRosVersionSetupBash(version).items():
            if k == 'PYTHONPATH' and sys.path.count(v) == 0:
                sys.path.append(v)
            elif not os.environ.has_key(k):
                os.environ[k] = v
            elif k.endswith('PATH') and os.environ[k].find(v) == -1:
                os.environ[k] = ':'.join((v, os.environ[k]))

        # if 'ROS_MASTER_URI' not in os.environ.keys():
        if rosMaster != None:
            os.environ['ROS_MASTER_URI'] = rosMaster

        path = '/opt/ros/%(version)s/ros' % { 'version': version}
        if 'ROS_ROOT' not in os.environ.keys() or os.environ['ROS_ROOT'] != path:
            os.environ['ROS_ROOT'] = path

        path = '%(root)s/bin' % { 'root': os.environ['ROS_ROOT']}
        if os.environ['PATH'].find(path) == -1:
            os.environ['PATH'] = ':'.join((path, os.environ['PATH']))

        path = '/opt/ros/%(version)s/stacks' % { 'version': version}
        if 'ROS_PACKAGE_PATH' not in os.environ.keys():
            os.environ['ROS_PACKAGE_PATH'] = path
        elif os.environ['ROS_PACKAGE_PATH'].find(path) == -1:
            os.environ['ROS_PACKAGE_PATH'] = ':'.join((path, os.environ['ROS_PACKAGE_PATH']))  

        if overlayPath != None:
            for path in overlayPath.split(':'):
                path = os.path.expanduser(path)
                if os.environ['ROS_PACKAGE_PATH'].find(path) == -1:
                    os.environ['ROS_PACKAGE_PATH'] = ':'.join((path, os.environ['ROS_PACKAGE_PATH']))

        path = packagePath or os.path.dirname(os.path.realpath(__file__)) + '/ROS_Packages'
        if os.environ['ROS_PACKAGE_PATH'].find(path) == -1:
            os.environ['ROS_PACKAGE_PATH'] = ':'.join((path, os.environ['ROS_PACKAGE_PATH']))

        path = '%(root)s/core/roslib/src' % { 'root': os.environ['ROS_ROOT']}
        if sys.path.count(path) == 0:
            sys.path.append(path)

        if packageName != None:
            with _threadLock:
                import roslib
                try:
                    roslib.load_manifest(packageName)
                except Exception as e:
                    print >> sys.stderr, "Error loading package: %s" % packageName
                    print >> sys.stderr, repr(e)

class RosSubscriber(object):
    """ Wrapper around a ros subscriber """
    """ Features: """
    """     Lazy initialisation """
    """     Automatic disconnection on idle, and reconnection on access set idleTime=None to disable"""
    """        Any call to hasNewMessage or lastMessage resets the idle timer """
    """     Async message retrieval (stores last message received) """
    def __init__(self, topic, dataType, idleTime=15):
        with _threadLock:
            ROS.configureROS(packageName='rospy')
            import rospy
            self._rospy = rospy
        self._lastAccess = time.time()
        self._subscriber = None
        self._topic = topic
        self._dataType = dataType
        self._newMessage = False
        self._idleTimeout = idleTime
        
    def __del__(self):
        self.unregister()

    @property
    def hasNewMessage(self):
        """ Return true if the message has changed since last call to hasNewMessage """
        """ Useful for topics which publish infrequently """
        self._touch()
        return self._newMessage
    
    @property
    def lastMessage(self):
        """ Return the last message received from the topic """
        self._touch()
        self._newMessage = False
        return self._data
    
    def _touch(self):
        """ Update class access time """
        self._lastAccess = time.time()
        if self._subscriber == None:
            with _threadLock:
                self._subscriber = self._rospy.Subscriber(self._topic, self._dataType, self._callback)
    
    def unregister(self):
        """ Unregister the subscriber """
        if self._subscriber != None:
            self._subscriber.unregister()
            self._subscriber = None
       
    def _callback(self, msg):
        """ Store message received from the topic, disconnect if class is idle """
        self._data = msg
        self._newMessage = True
        if self._idleTimeout != None and time.time() - self._lastAccess > self._idleTimeout:
            self.unregister()

class Transform(object):
    """ Wrapper around ros transform that handles common actions """
    def __init__(self, rosHelper=None, fromTopic=None, toTopic=None):
        """ fromTopic and toTopic are the defaults to use in getTransform """
        with _threadLock:
            if(rosHelper == None):
                self._ros = ROS()
            else:
                self._ros = rosHelper
            self._ros.configureROS(packageName='core_transform')
            import tf
            import rospy
            self._rospy = rospy
            self._tf = tf
            self._ros.initROS()

        self._listener = None
        self._defaultFrom = fromTopic
        self._defaultTo = toTopic
    
    def transformPoint(self, point, fromTopic=None, toTopic=None):
        """ Call ros transform between specified coordinate frames (or constructor defaults if None) """
        if fromTopic == None:
            fromTopic = self._defaultFrom
        if toTopic == None:
            toTopic = self._defaultTo
        
        """
        Waits for the /fromTopic to /toTopic transform to be availalble and 
        returns two tuples: (x, y, z) and a quaternion ( rx, ry, rz, rxy)
        Note: z values are 0 for 2D mapping and navigation.
        """
        if self._listener == None:
            self._listener = self._tf.TransformListener()

        # Wait for tf to get the frames
        with _threadLock:
            now = self._rospy.Time(0)
            try:
                self._listener.waitForTransform(toTopic, fromTopic, now, self._rospy.Duration(1.0))
            except self._tf.Exception as e:
                # if str(e) != 'Unable to lookup transform, cache is empty, when looking up transform from frame [' + baseTopic + '] to frame [' + mapTopic + ']':
                print >> sys.stderr, "Error while waiting for transform: " + str(e)
                return ((None, None, None), None)
        
        try:
            xyPos = self._listener.transformPoint(toTopic, point)
            return (xyPos.point.x, xyPos.point.y, xyPos.point.z)
        except (self._tf.LookupException, self._tf.ConnectivityException, self._tf.ExtrapolationException) as e:
            print >> sys.stderr, "Error while looking up transform: " + str(e)
            return (None, None, None)
    
    def getTransform(self, fromTopic=None, toTopic=None):
        """ Call ros transform between specified coordinate frames (or constructor defaults if None) """
        if fromTopic == None:
            fromTopic = self._defaultFrom
        if toTopic == None:
            toTopic = self._defaultTo
        
        """
        Waits for the /fromTopic to /toTopic transform to be availalble and 
        returns two tuples: (x, y, z) and a quaternion ( rx, ry, rz, rxy)
        Note: z values are 0 for 2D mapping and navigation.
        """
        if len(self._ros.getTopics('base_pose', exactMatch=True)) == 0:
            # this should work for all navigation systems, but at a performance cost
            if self._listener == None:
                self._listener = self._tf.TransformListener()

            # Wait for tf to get the frames
            with _threadLock:
                now = self._rospy.Time(0)
                try:
                    self._listener.waitForTransform(toTopic, fromTopic, now, self._rospy.Duration(1.0))
                except self._tf.Exception as e:
                    # if str(e) != 'Unable to lookup transform, cache is empty, when looking up transform from frame [' + baseTopic + '] to frame [' + mapTopic + ']':
                    print >> sys.stderr, "Error while waiting for transform: " + str(e)
                    return ((None, None, None), None)
            
            try:
                (xyPos, heading) = self._listener.lookupTransform(fromTopic, toTopic, now)
                (_, _, orientation) = self._tf.transformations.euler_from_quaternion(heading)
                return (xyPos, orientation)
            except (self._tf.LookupException, self._tf.ConnectivityException, self._tf.ExtrapolationException) as e:
                print >> sys.stderr, "Error while looking up transform: " + str(e)
                return ((None, None, None), None)
        else:
            # this takes significantly less processing time, but requires ipa_navigation    
            poseMsg = self._ros.getSingleMessage('/base_pose')
            if poseMsg == None:
                print >> sys.stderr, "No message recieved from /base_pose"
                return ((None, None, None), None)
            pose = poseMsg.pose
            xyPos = (pose.position.x, pose.position.y, pose.position.z)
            (_, _, orientation) = self._tf.transformations.euler_from_quaternion((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))
            return (xyPos, orientation)

if __name__ == '__main__':
    ROS.configureROS(version='electric', packagePath=None, packageName=None, rosMaster='http://sf1-1-pc1:11311', overlayPath=None)
    r = ROS()
    t = Transform(r)
    from Data.dataAccess import Locations
    from Data.dataAccess import DataAccess
    dao = DataAccess()
    while True:
        loc = t.getRobotPose()
        print Locations.resolveLocation(loc, None, dao)
        time.sleep(0.25)
    
    
    
