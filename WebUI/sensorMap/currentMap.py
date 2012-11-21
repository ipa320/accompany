import io, cherrypy, mimetypes
from cherrypy.lib import file_generator

from SensorMap.processor import MapProcessor
from Data.dataAccess import DataAccess
from Data.sensors import StateResolver
from Robots.careobot import CareOBot

class MapImage(object):
    exposed = True
    
    def __init__(self):
        self._robotName = CareOBot().name
        self._dao = DataAccess()
        self._sr = StateResolver()
        
    def GET(self, *args, **kwargs):
        #if len(args) < 1:
            #raise cherrypy.HTTPError(403, 'Directory Listing Denied')

        mp = MapProcessor()
        sensors = self._sr.resolveStates(self._dao.findSensors())
        sensors = self._sr.appendSensorMetadata(sensors) #adds location and type
        cob = self._dao.getRobotByName(self._robotName)
        robot = {
               'type':'robot', 
               'name':cob['robotName'], 
               'location': (cob['xCoord'], cob['yCoord'], '%sd' % (cob['orientation'] * -1)), #svg rotates opposite of our cooridnate system
               'id':'r%s' % (cob['robotId'])
               }

        elements = []
        elements.extend(sensors)
        #important to put robot last as z-order is determined by render order in svg and we want the robot icon
        #to always be on top
        elements.append(robot)
        
        img = mp.buildMap(elements)
        
        data = io.BytesIO(img)
        cherrypy.response.headers['Content-Type'] = mimetypes.guess_type('img.svg')[0]
        return file_generator(data)
