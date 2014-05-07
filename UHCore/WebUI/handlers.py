from cherrypy.lib import file_generator
import mimetypes
import cherrypy
import os

class Index(object):
    exposed = True

    def __init__(self, index='index.html'):
        self._index = index

    def GET(self, *args, **kwargs):
        if not cherrypy.request.path_info.endswith('/'):
            raise cherrypy.HTTPRedirect(cherrypy.request.path_info + '/')
        path = os.path.join(os.path.dirname(os.path.realpath(__file__)), self._index)
        f = open(path)
        text = f.read()
        f.close()
        cherrypy.response.headers['Content-Type'] = mimetypes.guess_type(path)[0]
        return text
    
    def PUT(self):
        pass

class StaticFile(object):
    exposed = True
    def __init__(self, fileName):
        self._file = fileName
    
    def GET(self, *args, **kwargs):
        if os.path.isfile(self._file):
            f = open(self._file, 'rb')
            cherrypy.response.headers['Content-Type'] = mimetypes.guess_type(self._file)[0]
            return file_generator(f)
        else:
            raise cherrypy.HTTPError(404)

class StaticFiles(object):
    exposed = True
    def __init__(self, rootDir):
        self._root = rootDir
    
    def GET(self, *args, **kwargs):
        path = ''
        for arg in args:
            path = os.path.join(path, arg)

        if path == '':
            raise cherrypy.HTTPError(403, 'Directory Listing Denied')
        
        ospath = os.path.join(self._root, path)
        if os.path.isfile(ospath):
            f = open(ospath, 'rb')
            cherrypy.response.headers['Content-Type'] = mimetypes.guess_type(ospath)[0]
            return file_generator(f)
        else:
            raise cherrypy.HTTPError(404)
