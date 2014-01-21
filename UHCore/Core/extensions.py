import time
from threading import Thread
""" Threading extensions that are used by continuous update processes """

class PollingThread(Thread):
    """ Thread extension used to execute a callback function with a given delay after each loop """
    def __init__(self, delayTime=0, group=None, target=None, completeCallback=None, name=None, args=(), kwargs=None, verbose=None):
        super(PollingThread, self).__init__(group=group, target=target, name=name, args=args, kwargs=kwargs, verbose=verbose)
        self._delayTime = delayTime
        self._completed = completeCallback
        self._cancelRequested = False
        self.daemon = True

    def cancel(self):
        self._cancelRequested = True
        
    def run(self):
        while not self._cancelRequested:
            if self._Thread__target:
                if self._Thread__args != None and self._Thread__kwargs != None:
                    self._Thread__target(*self._Thread__args, **self._Thread__kwargs)
                elif self._Thread__args != None:
                    self._Thread__target(*self._Thread__args, **{})
                elif self._Thread__kwargs != None:
                    self._Thread__target(*(), **self._Thread__kwargs)
                else:
                    self._Thread__target()
            
            if self._delayTime > 0:
                # To allow for cancellation, we don't want to block for the entire time
                # Check every 0.05 seconds for a cancel request
                sleep = 0.05
                delay = self._delayTime
                while delay > 0 and not self._cancelRequested:
                    time.sleep(sleep)
                    delay -= sleep

        if self._completed != None:
            self._completed(*self._Thread__args, **self._Thread__kwargs)

class PollingProcessor(object):
    """ Abstract base class for all continuous database updates """
    """ Manages calls PollingThreads """
    def __init__(self):
        self._runningThreads = {}
    
    def _removePollingProcessor(self, name):
        if self._runningThreads.has_key(name):
            lp = self._runningThreads[name]
            lp.cancel()
            lp.join(5)
            return True
        else:
            return False
    
    def _addPollingProcessor(self, name, function , args, delaySeconds):
        if not self._runningThreads.has_key(name):
            ahw = PollingThread(target=function, delayTime=delaySeconds, args=args)
            ahw.start()
            self._runningThreads[name] = ahw
        
        return name
