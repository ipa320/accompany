import io, cherrypy, mimetypes
from cherrypy.lib import file_generator

from SensorMap.processor import MapProcessor
from Data.dataAccess import DataAccess, Locations, Robots
from Data.stateResolver import StateResolver
from config import locations_config

class MapImage(object):
    exposed = True
    
    def __init__(self):
        #TODO: Handle multiple robots
        self._dao = DataAccess()
        self._sr = StateResolver()
        activeLocation = Locations().getActiveExperimentLocation()
        if activeLocation == None:
            return cherrypy.HTTPError(500, "Unable to determine active location")

        robot = Robots().getRobot(activeLocation['activeRobot'])
        self._robotName = robot['robotName']
        
        self._emptyMap = locations_config[activeLocation['location']]['map']
        
    def GET(self, *args, **kwargs):
        #if len(args) < 1:
            #raise cherrypy.HTTPError(403, 'Directory Listing Denied')

        mp = MapProcessor(self._emptyMap)
        sensors = self._sr.resolveStates(self._dao.findSensors())
        #sensors = self._sr.appendSensorMetadata(sensors, self._emptyMap) #adds location and type
        cob = self._dao.getRobotByName(self._robotName)
        robot = {
               'icon':self._dao.sensors.getSensorIconByName('robot')['id'], 
               'name':cob['robotName'], 
               'xCoord': cob['xCoord'],
               'yCoord': cob['yCoord'],
               'orientation': '%sd' % cob['orientation'],
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
