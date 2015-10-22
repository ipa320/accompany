from Data.dataAccess import DataAccess
from SensorMap.processor import MapProcessor
from Data.stateResolver import StateResolver

import cherrypy
from cherrypy.lib import file_generator

import io, mimetypes, json, os

class Root(object):
    exposed = True
    
    def __init__(self):
        self._index = 'actionHistory5thDesign.html'
    
    def GET(self, *args, **kwargs):
        if not cherrypy.request.path_info.endswith('/'):
            raise cherrypy.HTTPRedirect(cherrypy.request.path_info + '/')
        
        path = os.path.join(os.path.dirname(os.path.realpath(__file__)), self._index)
        f = open(path)
        text = f.read()
        f.close()
        
        cherrypy.response.headers['Content-Type'] = mimetypes.guess_type(path)[0]
        
        if not kwargs.has_key('tags'):
            return text
        else:
            #extremely dangerous and hacky way to do this...
            #return text.replace("var tags = ''", "var tags = '%s'" % kwargs['tags']);
            return text.replace("dao.getEvents('')", "dao.getEvents('', '%s')" % kwargs['tags'])
        
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
        elif kwargs.has_key('key'):
            dataKey = kwargs['key']
        else:
            dataKey = None

        obj = None    
        if dataType == 'user':
            obj = {}
        elif dataType == 'robot':
            obj = {}
        elif dataType == 'events':
            if kwargs.has_key('tags'):
                tags = kwargs['tags'].split(',')
            else:
                tags = ()
            obj = self.getEventsOrdered(dataKey, tags)
        else :
            raise cherrypy.HTTPError(400)
        
        cherrypy.response.headers['Content-Type'] = 'application/json'
        return json.dumps(obj)
    
    def POST(self, *args, **kwargs):
        request = json.loads(cherrypy.request.body.read())
        if not request.has_key('historyId'):
            raise cherrypy.HTTPError(400)
        else:
            historyId = request['historyId']
            tags = request['tags']
            if type(tags) != tuple and type(tags) != list:
                tags = (tags, )
            if type(tags) == list:
                tags = tuple(tags)
            if self._dao.actionHistory.updateTags(historyId, tags) >= 0:
                return 'OK'
            else:
                raise cherrypy.HTTPError(500)

    def getEventsOrdered(self, key, tags):
        hists = self._dao.getHistoryOrdered(key, tags)
        #if len(events) == 0:
        #    raise cherrypy.HTTPError(404)
        
        for hist in hists:
            if hist['scenario'] != None:
                #hist['gifThUrl'] = 'images/th_%s.gif' % hist['scenario']
                hist['imageArt'] = 'images/%s.jpg' % hist['imageArt']
            for event in hist['events']:
                if event['imageId'] != None:
                    event['imageUrl'] = 'images/%s' % event['imageId']
                    event['imageThUrl'] = 'images/th_r%s.jpg' % event['imageId']
                event.pop('imageId')
                if event['imageOverheadId'] != None:
                    event['imageOverheadUrl'] = 'imagesOverhead/%s' % event['imageOverheadId']
                event.pop('imageOverheadId')

        for hist in hists:
            filename = 'history/json/%s.json' %hist['eventId']
            with open(filename, 'wb') as outfile:
                json.dump(hist, outfile)

        return {'episodes': hists}

    def getEvents(self, key, tags):
        events = self._dao.getHistory(key, tags)
        #if len(events) == 0:
        #    raise cherrypy.HTTPError(404)
        
        for event in events:
            if event['imageId'] != None:
                event['imageThUrl'] = 'images/th_r%s.jpg' % event['imageId']
            event.pop('imageId')
            if event['imageOverheadId'] != None:
                event['imageOverheadUrl'] = 'imagesOverhead/%s' % event['imageOverheadId']
            event.pop('imageOverheadId')
            #if event['sensors'] != None and len(event['sensors']) > 0:
            #    event['sensorMapUrl'] = 'mapHistory/%s' % event['id']
            
        return {'Episodes': [{'Events': events}]}

class Images(object):
    exposed = True
    
    def __init__(self):
        self._dao = DataAccess()
        self._basePath = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'images')
        
    def GET(self, *args, **kwargs):
        if len(args) < 1:
            raise cherrypy.HTTPError(403, 'Directory Listing Denied')

        from PIL import Image
        fileName, fileExtension = os.path.splitext(args[0])
        if fileExtension=='.jpg':
            #path = os.path.join(self._basePath, args[0])
            data = Image.open(os.path.join(self._basePath, args[0]))
            img = {
                   'data': None,
                   'meta': {
                            'type': '',
                            'name': ''
                            }
                   }
        else:
            img = self._dao.getBinary(args[0])

        if img['data'] == None:
            path = os.path.join(self._basePath, args[0])
            if os.path.exists(path):
                data = io.FileIO(path)
            else:
                raise cherrypy.HTTPError(404)
        else:
            data = io.BytesIO(img['data'])

        cherrypy.response.headers['Content-Type'] = mimetypes.guess_type(img['meta']['name'] + '.' + img['meta']['type'])[0]
        return file_generator(data)

class ImagesOverhead(object):
    exposed = True
    
    def __init__(self):
        self._dao = DataAccess()
        self._basePath = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'imagesOverhead')
        
    def GET(self, *args, **kwargs):
        if len(args) < 1:
            raise cherrypy.HTTPError(403, 'Directory Listing Denied')

        img = self._dao.getBinaryOverhead(args[0])
        
        if img['data'] == None:
            path = os.path.join(self._basePath, args[0])
            if os.path.exists(path):
                data = io.FileIO(path)
            else:
                raise cherrypy.HTTPError(404)
        else:
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
            img = mp.buildMap(sensors)
            data = io.BytesIO(img)
            cherrypy.response.headers['Content-Type'] = mimetypes.guess_type('img.svg')[0]
            return file_generator(data)
