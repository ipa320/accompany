
class Sunflower(object):
    _imageFormats = ['BMP', 'EPS', 'GIF', 'IM', 'JPEG', 'PCD', 'PCX', 'PDF', 'PNG', 'PPM', 'TIFF', 'XBM', 'XPM']

    def __init__(self, name='Sunflower'):
        self._name = name

    @property
    def name(self):
        return self._name

    def getImage(self, leftRight='right', retFormat='PNG'):
        pass

    def executeFunction(self, funcName, kwargs):
        """Exectues an arbitrary function that the robot supports"""
        pass

    def getLocation(self, raw=False):
        """returns the robots current location"""
        pass

    def setLight(self, colour):
        pass

    def setComponentState(self, name, value):
        """set a component on the robot to the specified value"""
        """for example 'tray' 'up"""
        pass

    def getComponentPositions(self, componentName):
        """returns the list of available position(s) of a component"""
        """['up', 'down', etc]"""
        pass

    def getComponents(self):
        """gets a list of available component names ['tray', 'arm', etc]"""
        pass

    def getComponentState(self, componentName, raw=False):
        """gets the current component state, if raw==False, resolves the named state to one of the
           states returned from getComponentPositions(), or '' if not near to any named state"""
        pass
