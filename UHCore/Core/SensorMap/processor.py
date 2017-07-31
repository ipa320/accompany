import io, sys, os, math, copy
from Data.dataAccess import Sensors
from xml.etree import ElementTree as et
from threading import RLock

class MapProcessor(object):
    """ Used to build the live image of the sensor network """
    _mapCache = {}
    _iconCache = {}
    _mapLock = RLock()
    _iconLock = RLock()

    def __init__(self, _map):
        self._root = os.path.dirname(os.path.realpath(__file__))
        self._baseFile = os.path.join(self._root, _map['base'])
        self._map = _map
        self._sensorTypes = {}
        self._dao = Sensors()

    @property
    def mapBase(self):
        """ Returns a copy of the base svg map file """
        """ Caches initial map for fast access in subsequent calls """
        if not MapProcessor._mapCache.has_key(self._baseFile):
            MapProcessor._mapLock.acquire()
            try:
                MapProcessor._mapCache[self._baseFile] = et.parse(self._baseFile)
            finally:
                MapProcessor._mapLock.release()
        
        return copy.deepcopy(MapProcessor._mapCache[self._baseFile])
    
    def getIcon(self, iconId, sensorOn=False):
        """ Returns the sensor icon (with option 'On' graphic') wrapped in a group node """
        """ Caches icon for fast access in subsequent calls """
        key = str(iconId) + str(sensorOn)
        if not MapProcessor._iconCache.has_key(key):
            MapProcessor._iconLock.acquire()
            try:
                if not self._sensorTypes.has_key(iconId):
                    self._sensorTypes[iconId] = self._dao.getSensorIcon(iconId) 
                    
                sensorDef = self._sensorTypes[iconId]
                
                imgFile = None
                imgPath = None
                imgName = None
                if sensorDef != None:
                    imgName = sensorDef['name'] 
                    if sensorDef['icon'] != None:
                        if sensorOn:
                            imgPath = sensorDef['icon'] + '_on' + ".svg"
                        else:
                            imgPath = sensorDef['icon'] + ".svg"
        
                        try:
                            imgFile = et.parse(os.path.join(self._root, imgPath))      
                        except Exception as e:
                            if sensorOn:
                                print >> sys.stderr, "Error parsing sensor image (%(path)s): %(error)s" % {'error' :e, 'path': imgPath }
    
                if imgFile == None:
                    if imgPath != None:
                        print "Unable to load image from %(path)s, using default" % {'path' : imgPath }
                    else:
                        print "Unable to load image for %(type)s, using default" % {'type': imgName }
                    imgPath = 'icons/default.svg'
                    imgFile = et.parse(os.path.join(self._root, imgPath))
                    imgFile.find('{http://www.w3.org/2000/svg}text').text = imgName            
                
                if sys.version_info >= (2, 7):
                    group = et.Element('g')
                else:
                    group = et.Element('ns0:g')
    
                for child in imgFile.getroot().getchildren():
                    group.append(et.fromstring(et.tostring(child)))
                
                height = float(imgFile.getroot().get('height', 0))
                width = float(imgFile.getroot().get('width', 0))
                
                MapProcessor._iconCache[key] = (group, height, width)
            finally:
                MapProcessor._iconLock.release()
        
        return copy.deepcopy(MapProcessor._iconCache[key])

    def buildMap(self, elements=[]):
        """elements=[{'state':'on', 'location':{'xCoord':2.3, 'yCoord':9.2', 'orientation':3.141}, 'id':12]"""
        """state must match from the sensor state=>sensor Icon mapping"""
        """state can be empty or missing to use a stateless icon"""
        """x and y are in meters"""
        """orientation is assumed in radians, use d or r suffix to use others (90d/6R)"""
        # TODO: when map is clicked, show RH coords
        
        if sys.version_info >= (2, 7):
            et.register_namespace("", "http://www.w3.org/2000/svg")

        root = self.mapBase.getroot()
        mapHeight = float(root.attrib['height'])
        cc = CoordinateConvertor(self._map)
        
        for element in elements:
            try:
                if element['on'] == None:
                    state = False
                else:
                    state = element['on']
            except:
                state = False
            (x, y, d) = cc.toSensorMap((element['xCoord'], element['yCoord'], element['orientation']))
            (img, height, width) = self.getIcon(element['icon'], state)

            # y is reversed for translation, seems that way at least
            My = mapHeight - y
            # be sure to translate first, which changes the local coordinate space to the group object
            # which is important for the rotation about the centre
            transform = "translate(%(x)s, %(y)s) rotate(%(rotate)s, %(xCenter)s, %(yCenter)s)" % {
                                                                                                  'x': x,
                                                                                                  'y': My,
                                                                                                  'rotate': d,
                                                                                                  'xCenter': (width / 2),
                                                                                                  'yCenter': (height / 2)
                                                                                                  }

            img.attrib['transform'] = transform
            img.attrib['id'] = str(element['id'])
            root.append(img)
         
        # ElementTree.write() doesn't write the headers
        f = io.BytesIO()
        f.write('<?xml version=\"1.0\" standalone=\"no\"?>\n')
        f.write('<!DOCTYPE svg PUBLIC \"-//W3C//DTD SVG 1.1//EN\" \"http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd\">\n')
        if sys.version_info >= (2, 7):
            f.write(et.tostring(root))
        else:
            f.write(et.tostring(root).replace('ns0:', '').replace(':ns0', '')) 
        ret = f.getvalue()
        f.close()
        return ret
    
class CoordinateConvertor(object):
    """Convert between ROS MAP and SVG MAP coordinate systems"""
    
    def __init__(self, transform):
        # This could be solved and combined into a single transformation,
        # but I left it this way so any changes to an individual transform could be 
        # easily corrected for
        self._sensorMapToRHMapScale = transform['scale']
        self._sensorMapToRHMapOffset = transform['offset']  # in sensorMapUnits
        self._sensorMapToRHLocRotation = transform['rotation']

        # from the map.yaml file, combining these gets from real world coordinates to pgm pixels
        self._RHMapToRHLocScale = 0.05 
        self._RHMapToRHLocOffset = (-8, -19.2)  # in RHLoc units
        
    def toRobotHouse(self, (Mx, My, Mr)):
        """x and y are in sensor map pixels"""
        """r is assumed in degrees, use d or r suffix to use others (90d/3.14r)"""
        
        #Defaults for missing values
        Mx = Mx or 0
        My = My or 0
        Mr = Mr or 0

        # (px*s)+cx if cx in final units
        # (px+cx)*s if cx in original units
        Rx = (Mx * self._sensorMapToRHMapScale) + self._sensorMapToRHMapOffset[0]
        Ry = (My * self._sensorMapToRHMapScale) + self._sensorMapToRHMapOffset[1]
        RHx = (Rx * self._RHMapToRHLocScale) + self._RHMapToRHLocOffset[0]
        RHy = (Ry * self._RHMapToRHLocScale) + self._RHMapToRHLocOffset[1]
        
        RHr = None
        if type(Mr) == str:            
            if Mr.endswith('r'):
                Mr = RHr.strip('r')
            
            if Mr.endswith('d'):
                RHr = float(RHr.strip('d'))
        
        if RHr == None:
            try:
                RHr = math.radians(Mr + self._sensorMapToRHLocRotation)
            except:
                RHr = None
        
        return (RHx, RHy, RHr)
        
    def toSensorMap(self, (RHx, RHy, RHr)):
        """x and y are in meters"""
        """r is assumed in radians, use d or r suffix to use others (90d/3.14r)"""
        
        #Defaults for missing values
        RHx = RHx or 0
        RHy = RHy or 0
        RHr = RHr or 0

        # convert from real world to PGM
        # (x - -8) / 0.05, (y - -19.2) / 0.05
        
        # convert from PGM to SVG
        # (x / 0.275) - 295, (y / 0.275) - 505
        
        Rx = (RHx - self._RHMapToRHLocOffset[0]) / self._RHMapToRHLocScale
        Ry = (RHy - self._RHMapToRHLocOffset[1]) / self._RHMapToRHLocScale
        Mx = (Rx - self._sensorMapToRHMapOffset[0]) / self._sensorMapToRHMapScale
        My = (Ry - self._sensorMapToRHMapOffset[1]) / self._sensorMapToRHMapScale
        RHr = str(RHr).lower()
        Mr = None
        if type(RHr) == str:
            try:
                if RHr.endswith('r'):
                    Mr = math.degrees(float(RHr.strip('r')))
                if RHr.endswith('d'):
                    Mr = float(RHr.strip('d')) 
            except:
                Mr = 0

        Mr = Mr * -1 #svg rotates opposite of our cooridnate system
        Mr = Mr - self._sensorMapToRHLocRotation
        return (Mx, My, Mr)

if __name__ == '__main__':
    cc = CoordinateConvertor()
    senLoc = cc.toSensorMap((0, 0, math.pi / 2))
    print senLoc
    rhLoc = cc.toRobotHouse(senLoc)
    print rhLoc
