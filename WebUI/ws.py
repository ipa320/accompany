#Add project reference
import sys, os
path = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../' + 'Core')
sys.path.append(path)

import cherrypy
import history
import questions
import sensorMap
import sensorDetails
import listRoot

from config import server_config

conf = {
    'global': {
        'server.socket_host': '0.0.0.0',
        'server.socket_port': server_config['http_port'],
        'environment': 'production'
    },
    '/': {
        'request.dispatch': cherrypy.dispatch.MethodDispatcher(),
    }
}

root = listRoot.root
listRoot.setLinks((
              ('history', history), 
              ('question', questions),
              ('details', sensorDetails),
              ('liveMap', sensorMap)))

#from processor import RobotLocationProcessor
#lp = RobotLocationProcessor()
#lp.startLocationPolling()

cherrypy.quickstart(root, '/', conf)

#lp.stopLocationPolling()