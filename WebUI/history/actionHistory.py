from Data.dataAccess import DataAccess
from SensorMap.processor import MapProcessor
from Data.sensors import StateResolver

import cherrypy
from cherrypy.lib import file_generator

import io, mimetypes, json

class Data(object):
    exposed = True
    
    def __init__(self):
        self._dao = DataAccess()
        
    def GET(self, *args, **kwargs):
        if len(args) < 1:
            raise cherrypy.HTTPError(400)
        dataType = args[0]
        if len(args) > 1:
            dataKey = args[1]
        else:
            dataKey = None

        obj = None    
        if dataType == 'user':
            obj = {}
        elif dataType == 'robot':
            obj = {}
        elif dataType == 'events':
            obj = self.getEvents(dataKey)
        else :
            raise cherrypy.HTTPError(400)
        
        cherrypy.response.headers['Content-Type'] = 'application/json'
        return json.dumps(obj)

    def getEvents(self, key):
        events = self._dao.getHistory(key)
        if len(events) == 0:
            raise cherrypy.HTTPError(404)
        
        for event in events:
            if event['imageId'] != None:
                event['imageUrl'] = 'images/%s' % event['imageId']
            event.pop('imageId')
            if event['sensors'] != None and len(event['sensors']) > 0:
                event['sensorMapUrl'] = 'mapHistory/%s' % event['id']
            
        return {'Episodes': [{'Events': events}]}

class Images(object):
    exposed = True
    
    def __init__(self):
        self._dao = DataAccess()
        
    def GET(self, *args, **kwargs):
        if len(args) < 1:
            raise cherrypy.HTTPError(403, 'Directory Listing Denied')

        img = self._dao.getBinary(args[0])
        
        if img['data'] == None:
            raise cherrypy.HTTPError(404)
        
        data = io.BytesIO(img['data'])
        cherrypy.response.headers['Content-Type'] = mimetypes.guess_type(img['meta']['name'] + '.' + img['meta']['type'])[0]
        return file_generator(data)

class MapHistory(object):
    exposed = True
    
    def __init__(self):
        self._dao = DataAccess()
        
    def GET(self, *args, **kwargs):
        if len(args) < 1:
            raise cherrypy.HTTPError(403, 'Directory Listing Denied')

        key = args[0]
        
        sensorHist = self._dao.getSensorHistory(key)        
        if len(sensorHist) > 0:
            mp = MapProcessor()
            s = StateResolver()
            sensors = s.resolveStates(sensorHist)
            #[{'id': sensor['sensorId'], 'value': sensor['value'], 'state':'on'},]
            sensors = s.appendSensorMetadata(sensors) #adds location and type            
            img = mp.buildMap(sensors)
            data = io.BytesIO(img)
            cherrypy.response.headers['Content-Type'] = mimetypes.guess_type('img.svg')[0]
            return file_generator(data)
