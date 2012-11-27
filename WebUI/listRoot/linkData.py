import cherrypy
import json

class LinkData(object):
    exposed = True
    
    def __init__(self, links):
        self._links = links
        #self._otherLinks = []
        
    def GET(self, *args, **kwargs):
        obj = []
        for (path, webClass) in self._links:
            if type(webClass) == str:
                name = webClass
            else:
                name = webClass.name
            obj.append(
                         {
                          'path': path,
                          'title': name
                          })
        
        #obj.extend(self._otherLinks)
        
        cherrypy.response.headers['Content-Type'] = 'application/json'
        return json.dumps(obj)
    
    #def addLink(self, title, path):
    #    self._otherLinks.append({'path': path, 'title': title});