#Add project reference
import sys, os
path = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../' + 'Core')
sys.path.append(path)

#temporary launch until the sienaGUI supports full paths
import cherrypy
import sienagui

from config import server_config

conf = {
    'global': {
        'server.socket_host': '0.0.0.0',
        'server.socket_port': server_config['http_port'] + 1,
        'server.thread_pool': 10,
        'server.thread_pool_max': -1,
    },
    '/': {
        'request.dispatch': cherrypy.dispatch.MethodDispatcher(),
    }
}

root = sienagui.root

cherrypy.quickstart(root, '/', conf)

