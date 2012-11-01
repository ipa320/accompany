import time
from threading import Thread

class PollingThread(Thread):
    
    def __init__(self, delayTime=0, group=None, target=None, completeCallback=None, name=None, args=(), kwargs=None, verbose=None):
        self._delayTime = delayTime
        self._completed = completeCallback
        self._cancelRequested = False
        super(PollingThread, self).__init__(group=group, target=target, name=name, args=args, kwargs=kwargs, verbose=verbose)

    def cancel(self):
        self._cancelRequested = True
    
    def run(self):
        while not self._cancelRequested:
            if self._Thread__target:
                self._Thread__target(*self._Thread__args, **self._Thread__kwargs)
            
            if self._delayTime > 0:
                delay = self._delayTime
                while delay > 0 and not self._cancelRequested:
                    time.sleep(0.1)
                    delay -= 0.1
                    
        if self._completed != None:
            self._completed(*self._Thread__args, **self._Thread__kwargs)
