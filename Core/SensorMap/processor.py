import io, sys, os, math, copy
from xml.etree import ElementTree as et
from threading import RLock

class MapProcessor(object):
    _mapCache = {}
    _iconCache = {}
    _mapLock = RLock()
    _iconLock = RLock()

    def __init__(self, baseMap='RobotHouseMap.svg'):
        self._root = os.path.dirname(os.path.realpath(__file__))
        self._baseFile = os.path.join(self._root, baseMap)
        self._sensorTypes = et.parse(os.path.join(self._root, 'type_icons.xml'))

    @property
    def mapBase(self):
        if not MapProcessor._mapCache.has_key(self._baseFile):
            MapProcessor._mapLock.acquire()
            try:
                MapProcessor._mapCache[self._baseFile] = et.parse(self._baseFile)
            finally:
                MapProcessor._mapLock.release()
        
        return copy.deepcopy(MapProcessor._mapCache[self._baseFile])
    
    def getIcon(self, sensorType, sensorOn=False):
        key = sensorType + str(sensorOn)
        if not MapProcessor._iconCache.has_key(key):
            MapProcessor._iconLock.acquire()
            try:
                rhSensorDef = None
                if sys.version_info >= (2, 7):
                    rhSensorDef = self._sensorTypes.find('type[@name="%s"]' % (sensorType))
                else:
                    for sensor in self._sensorTypes.findall('type'):
                        if sensor.attrib['name'] == sensorType:
                            rhSensorDef = sensor
                            break
                
                imgFile = None
                imgPath = None
                if rhSensorDef != None and rhSensorDef.attrib.has_key('image'):
                    if sensorOn:
                        imgPath = rhSensorDef.attrib['image'] + '_on' + ".svg"
                    else:
                        imgPath = rhSensorDef.attrib['image'] + ".svg"
    
                    try:
                        imgFile = et.parse(os.path.join(self._root, imgPath))      
                    except Exception as e:
                        if sensorOn:
                            print >> sys.stderr, "Error parsing %(name)s sensor image: %(error)s" % {'error' :e, 'name': sensorType }
    
                if imgFile == None:
                    if imgPath != None:
                        print "Unable to load image from %(path)s, using default" % {'path' : imgPath }
                    else:
                        print "Unable to load image for %(type)s, using default" % {'type': sensorType }
                    imgPath = 'icons/default.svg'
                    imgFile = et.parse(os.path.join(self._root, imgPath))
                    imgFile.find('{http://www.w3.org/2000/svg}text').text = sensorType            
                
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
        #TODO: when map is clicked, show RH coords
        
        if sys.version_info >= (2, 7):
            et.register_namespace("", "http://www.w3.org/2000/svg")

        root = self.mapBase.getroot()
        mapHeight = float(root.attrib['height'])
        cc = CoordinateConvertor()
        
        for element in elements:
            if element.has_key('on'):
                state = element['on']
            else:
                state = False
            (x, y, d) = cc.toSensorMap(element['location'])
            (img, height, width) = self.getIcon(element['type'], state)

            #y is reversed for translation, seems that way at least
            My = mapHeight - y
            #be sure to translate first, which changes the local coordinate space to the group object
            #which is important for the rotation about the centre
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
        if sys.version_info >= (2,7):
            f.write(et.tostring(root))
        else:
            f.write(et.tostring(root).replace('ns0:', '').replace(':ns0', '')) 
        ret = f.getvalue()
        f.close()
        return ret
    
class CoordinateConvertor(object):
    """Convert between Robot House and SensorMap coordinate systems"""
    
    def __init__(self):
        #This could be solved and combined into a single transformation,
        #but I left it this way so any changes to an individual transform could be 
        #easily corrected for
        self._sensorMapToRHMapScale = 0.275
        self._sensorMapToRHMapOffset = (81, 245) #in sensorMapUnits
        self._sensorMapToRHLocRotation = -90

        #from the map.yaml file
        self._RHMapToRHLocScale = 0.05
        self._RHMapToRHLocOffset = (-8, -19.2) #in RHLoc units
        
    def toRobotHouse(self, (Mx, My, Mr)):
        """x and y are in sensor map pixels"""
        """r is assumed in degrees, use d or r suffix to use others (90d/6R)"""
        #(px*s)+cx if cx in final units
        #(px+cx)*s if cx in original units
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
        """r is assumed in radians, use d or r suffix to use others (90d/6R)"""
        Rx = (RHx - self._RHMapToRHLocOffset[0]) / self._RHMapToRHLocScale
        Ry = (RHy - self._RHMapToRHLocOffset[1]) / self._RHMapToRHLocScale
        Mx = (Rx - self._sensorMapToRHMapOffset[0]) / self._sensorMapToRHMapScale
        My = (Ry - self._sensorMapToRHMapOffset[1]) / self._sensorMapToRHMapScale
        RHr = str(RHr).lower()
        Mr = None
        if type(RHr) == str:            
            if RHr.endswith('r'):
                RHr = RHr.strip('r')
            
            if RHr.endswith('d'):
                Mr = float(RHr.strip('d')) - self._sensorMapToRHLocRotation
        
        if Mr == None:
            try:
                Mr = math.degrees(float(RHr)) - self._sensorMapToRHLocRotation
            except:
                Mr = None
        return (Mx, My, Mr)

if __name__ == '__main__':
    cc = CoordinateConvertor()
    senLoc = cc.toSensorMap((0,0,math.pi / 2))
    print senLoc
    rhLoc = cc.toRobotHouse(senLoc)
    print rhLoc