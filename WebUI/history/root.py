import handlers, os
from actionHistory import MapHistory, Data, Images, Root

name = "Action History"

root = Root()
root.data = Data()
root.images = Images()
root.mapHistory = MapHistory()
root.iui = handlers.StaticFiles(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'iui'))
root.js = handlers.StaticFiles(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'js'))
