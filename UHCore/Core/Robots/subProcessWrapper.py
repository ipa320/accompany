from multiprocessing import Process, Pipe, RLock
import sys
import traceback

class SubProcessWrapper(Process):
    def __init__(self, imports, className, *args, **kwargs):
        super(SubProcessWrapper, self).__init__()
        self._sendPipe, self._recvPipe = Pipe()
        self._class = {
                       'imports': imports,
                       'name': className,
                       'args': args,
                       'kwargs': kwargs
                       }
        self._pipeLock = RLock()
        self._stop = False
        self.daemon = True
        Process.start(self)
    
    def __del__(self):
        self._sendPipe.close()
        self._recvPipe.close()
        
    def __getattr__(self, name):
        if name in object.__getattribute__(self, '__dict__'):
            return object.__getattribute__(self, name)
        def wrappedFunc(*args, **kwargs):
            msg = {'name': name, 'args':args, 'kwargs':kwargs}
            return object.__getattribute__(self, '_send')(msg)
        return wrappedFunc
    
    def run(self):
        # everything inside here is run in a separate process space
        try:
            for importStatement in self._class['imports']:
                if importStatement.startswith('import '):
                    for namespace in importStatement[7:].split(','):
                        globals()[namespace.strip()] = __import__(namespace.strip(), globals(), locals())
                elif importStatement.startswith('from '):
                    # from time import sleep
                    # from time import sleep, clock as ticker
                    namespace = importStatement[5: importStatement.index(' ', 5)]
                    fromlist = importStatement[importStatement.index(' ', 5) + 7:].strip()
                    for item in fromlist.split(','):
                        if item.strip() == '':
                            continue
                        if ' as ' in item:
                            itemName = item[:item.index(' as ')].strip()
                            asName = item[item.index(' as ') + 3:].strip()
                            temp = __import__(namespace, globals(), locals(), [itemName])
                            globals()[asName] = getattr(temp, itemName)
                        else:
                            temp = __import__(namespace, globals(), locals(), [item])
                            attr = getattr(temp, item)
                            globals()[item] = attr
                else:
                    raise ValueError("Unknown import statement '%s'" % importStatement)
        except Exception as e:
            print >> sys.stderr, "%s\n%s" % (e, traceback.format_exc(e))
            self._recvPipe.send(e)
            return
        
        if self._class['name'] in globals():
            classInstance = globals()[self._class['name']](*self._class['args'], **self._class['kwargs'])
        else:
            raise ValueError("Unable to create class %s in from current imports" % (self._class['name'], ))
        while True:
            try:
                msg = self._recvPipe.recv()
            except EOFError:
                break
            except KeyboardInterrupt:
                break
            
            if not hasattr(classInstance, msg['name']):
                ret = AttributeError("'%s' object has no attribute '%s'" % (self._class['name'], msg['name']))
            else:
                try:
                    func = getattr(classInstance, msg['name'])
                    ret = func(*msg.get('args', ()), **msg.get('kwargs', {}))
                except Exception as e:
                    print >> sys.stderr, "%s\n%s" % (e, traceback.format_exc())
                    ret = e
            self._recvPipe.send(ret)
            
    def _send(self, msg):
        self._pipeLock.acquire()
        try:
            self._sendPipe.send(msg)
            try:
                ret = self._sendPipe.recv()
            except EOFError:
                return None
            
            if isinstance(ret, Exception):
                raise ret
            else:
                return ret
        except EOFError:
            return
        except KeyboardInterrupt:
            return
        finally:
            self._pipeLock.release()
