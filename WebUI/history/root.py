import handlers, os
from actionHistory import MapHistory, Data, Images

name = "Action History"

root = handlers.Index(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'actionHistory.html'))
root.data = Data()
root.images = Images()
root.mapHistory = MapHistory()
root.iui = handlers.StaticFiles(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'iui'))
root.js = handlers.StaticFiles(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'js'))
