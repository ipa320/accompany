from multiprocessing import Process, Pipe, RLock
import os

import rosHelper

class ROSMulti(object):
    """ Experimental ros interface wrapping the complexities of communicating with multiple ros masters """
    """ Creates a new process, configured with the appropriate variables and uses pipes to relay data """
    """ to and from the current process.  The hope is to stablise this to the point that it can """
    """ replace direct calls to rosHelper, allowing for communication and control of multiple robots """
    """ from the same top level process. """
    def __init__(self, version=None, packagePath=None, packageName=None, rosMaster=None, overlayPath=None):
        localPipe, remotePipe = Pipe()
        self._pipe = localPipe
        self._rosRemote = _RosMulti(version, rosMaster, overlayPath, remotePipe)
        self._rosRemote.start()
        self._pipeLock = RLock()
    
    def __del__(self):
        self._pipe.close()
    
    def getSingleMessage(self, topic, retryOnFailure=1, timeout=None):
        msg = Message('getSingleMessage', {'topic':topic, 'retryOnFailure':retryOnFailure, 'timeout':timeout})
        return self._send(msg)
        
    def getTopics(self, baseFilter='', exactMatch=False):
        msg = Message('getTopics', {'baseFilter':baseFilter, 'exactMatch':exactMatch})
        return self._send(msg)

    def getMessageType(self, topic):
        msg = Message('getMessageType', {'topic':topic})
        return self._send(msg)
    
    def getParam(self, paramName):
        msg = Message('getParam', {'paramName': paramName})
        return self._send(msg)
    
    def _send(self, msg):
        self._pipeLock.acquire()
        try:
            self._pipe.send(msg)
            try:
                ret = self._pipe.recv()
            except EOFError:
                return None
            
            if type(ret) == Exception:
                raise ret
            else:
                return ret
        finally:
            self._pipeLock.release()

class Transform(rosHelper.Transform):
    def __init__(self, rosHelper=None, fromTopic=None, toTopic=None):
        if(rosHelper == None):
            self._ros = ROS()
        else:
            self._ros = rosHelper
        super(Transform, self).__init__(rosHelper, fromTopic, toTopic)
        
class _RosMulti(Process):
    def __init__(self, version, rosMaster, overlayPath, pipe):
        super(_RosMulti, self).__init__()
        self._version = version
        self._rosMaster = rosMaster
        self._overlayPath = overlayPath
        self._ros = None
        self._pipe = pipe
        self._cancel = False
    
    def __del(self):
        self._pipe.close()
    
    def run(self):
        # everthing inside here is run in a separate process space
        os.environ = rosHelper.ROS.parseRosBash('electric', False)
        rosHelper.ROS.configureROS(version=self._version, rosMaster=self._rosMaster, overlayPath=self._overlayPath)
        self._ros = rosHelper.ROS()
        while True:
            try:
                msg = self._pipe.recv()
            except EOFError:
                break
            
            try:
                func = getattr(self._ros, msg.funcName)
                ret = func(*(), **msg.kwargs)
            except Exception as e:
                ret = e
            self._pipe.send(ret)

class Message(object):
    def __init__(self, funcName, kwargs):
        self._func = funcName
        self._kwargs = kwargs
        
    @property
    def funcName(self):
        return self._func
    
    @property
    def kwargs(self):
        return self._kwargs

if __name__ == '__main__':
    r = ROS()
    print r.getTopics('asdf')
    r = None
