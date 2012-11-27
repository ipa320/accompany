import time, os, sys

class ROS(object):
    def __init__(self, *args, **kwargs):
        ROS.configureROS(packageName='rospy')
        import rospy
        self._rospy = rospy
        
    def initROS(self, name='rosHelper'):
        if not self._rospy.core.is_initialized():
            self._rospy.init_node('rosHelper', anonymous=True, disable_signals=True)
        
    def getSingleMessage(self, topic, dataType=None, retryOnFailure=1, timeout=None):
        
        try:
            if dataType == None:
                dataType = self.getMessageType(topic)
            
            callback = RosCallback()
            self.initROS()
            subscriber = self._rospy.Subscriber(topic, dataType, callback.callback)
            
            while not callback.done:
                if timeout != None:
                    if timeout < 0:
                        break
                    else:
                        timeout -= 0.01
                time.sleep(0.01)                
    
            subscriber.unregister()

            return callback.data
        except:
            if retryOnFailure > 0:
                return self.getSingleMessage(topic, dataType, retryOnFailure - 1, timeout)
            else:
                return None
    
    def getTopics(self, baseFilter=''):
        topics = self._rospy.get_published_topics(baseFilter)
        return topics

    def getMessageType(self, topic):
        controller_name = topic[0: topic.rfind('/')]
        controller_msgType = None
        
        topics = self.getTopics(controller_name)
        for pubTopic in topics:
            if pubTopic[0] == topic:
                controller_msgType = pubTopic[1]
                break
        
        if controller_msgType == None:
            raise Exception('Could not determine ROS messageType for topic: %s' % (topic))
        
        manifest = controller_msgType.split('/')[0]
        cls = controller_msgType.split('/')[1]
        
        try:
            import roslib
            roslib.load_manifest(manifest)
            
            ns = __import__(manifest + '.msg', globals(), locals(), [cls], -1)
            msgCls = getattr(ns, cls)
            return msgCls
        except Exception as e:
            raise Exception('Error occured while loading message class: %s' % (e))
        
    @staticmethod
    def configureROS(version=None, packagePath=None, packageName=None, rosMaster=None, overlayPath=None):
        """Any values not provided will be read from ros_config in config.py"""
        
        from config import ros_config
        version = version or ros_config['version']
        rosMaster = rosMaster or ros_config['rosMaster']
        overlayPath = overlayPath or ros_config['overlayPath']
        
        if 'ROS_MASTER_URI' not in os.environ.keys():
            os.environ['ROS_MASTER_URI'] = rosMaster

        if 'ROS_ROOT' not in os.environ.keys():
            os.environ['ROS_ROOT'] = '/opt/ros/%(version)s/ros' % { 'version': version}

        path = '%(root)s/bin' % { 'root': os.environ['ROS_ROOT']}
        if os.environ['PATH'].find(path) == -1:
            os.environ['PATH'] = ':'.join((path, os.environ['PATH']))

        path = '/opt/ros/%(version)s/stacks:' % { 'version': version}
        if 'ROS_PACKAGE_PATH' not in os.environ.keys():
            os.environ['ROS_PACKAGE_PATH'] = path
        elif os.environ['ROS_PACKAGE_PATH'].find(path) == -1:
            os.environ['ROS_PACKAGE_PATH'] = ':'.join((path, os.environ['ROS_PACKAGE_PATH']))  

        path = os.path.expanduser(overlayPath)
        if os.environ['ROS_PACKAGE_PATH'].find(path) == -1:
            os.environ['ROS_PACKAGE_PATH'] = ':'.join((path, os.environ['ROS_PACKAGE_PATH']))

        path = packagePath or os.path.dirname(os.path.realpath(__file__)) + '/ROS_Packages'
        if os.environ['ROS_PACKAGE_PATH'].find(path) == -1:
            os.environ['ROS_PACKAGE_PATH'] = ':'.join((path, os.environ['ROS_PACKAGE_PATH']))

        path = '%(root)s/core/roslib/src' % { 'root': os.environ['ROS_ROOT']}
        if sys.path.count(path) == 0:
            sys.path.append(path)

        if packageName != None:
            import roslib
            roslib.load_manifest(packageName)

class RosCallback(object):
     
    def __init__(self):
        self._done = False
        self._message = None
         
    def callback(self, msg):
        self._message = msg
        self._done = True
        
    @property
    def done(self):
        return self._done
    
    @property
    def data(self):
        return self._message


if __name__ == '__main__':
    import os
    r = ROS()
    r.configureROS()
    print os.environ['ROS_PACKAGE_PATH']
