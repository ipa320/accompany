from Data.dataAccess import DataAccess

import cherrypy
import json

class Data(object):
    exposed = True
    
    def __init__(self):
        self._dao = DataAccess()
        
    def GET(self, *args, **kwargs):
        if len(args) < 1:
            raise cherrypy.HTTPError(400)
        
        questionNo = args[0]
        if questionNo == 'current':
            ques = self._dao.getActiveQuestion()
            if ques != None:
                questionNo = ques['sequenceName']
            else:
                questionNo = None
        
        if questionNo != None:
            resp = self._dao.getResponses(questionNo)
            obj = {'query': questionNo, 'responses':resp}
        else:
            obj = {'query': 'none'}
        
        cherrypy.response.headers['Content-Type'] = 'application/json'
        return json.dumps(obj)

    def POST(self, *args, **kwargs):
        request = json.loads(cherrypy.request.body.read())
        if not request.has_key('response'):
            raise cherrypy.HTTPError(400)
        else:
            userresponse = int(request['response'])
            if self._dao.setResponse(args[0], userresponse):
                return 'OK'
            else:
                raise cherrypy.HTTPError(500)
