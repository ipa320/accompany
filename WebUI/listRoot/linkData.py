import cherrypy
import json

class LinkData(object):
    exposed = True
    
    def __init__(self, links):
        self._links = links
        
    def GET(self, *args, **kwargs):
        obj = []
        for (path, webClass) in self._links:
            obj.append(
                         {
                          'path': path,
                          'title': webClass.name
                          })
        
        cherrypy.response.headers['Content-Type'] = 'application/json'
        return json.dumps(obj)