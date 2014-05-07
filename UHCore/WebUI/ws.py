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
#import sienagui

from config import server_config

conf = {
    'global': {
        'server.socket_host': '0.0.0.0',
        'server.socket_port': server_config['http_port'],
        'server.thread_pool': 10,
        'server.thread_pool_max': -1,
        #'environment': 'production'
    },
    '/': {
        'request.dispatch': cherrypy.dispatch.MethodDispatcher(),
    }
}

root = listRoot.root
listRoot.setLinks((
              ('history', history),
              ('history/?tags=important', 'Important History'),
              ('history/?tags=question', 'Unclear History'),
              ('question', questions),
              ('details', sensorDetails),
              ('liveMap', sensorMap)))

#root.sienagui = sienagui.root

cherrypy.quickstart(root, '/', conf)
