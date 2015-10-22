import handlers, os
from actionHistory import MapHistory, Data, Images, ImagesOverhead, Root

name = "Action History"

root = Root()
root.data = Data()
root.images = Images()
root.imagesOverhead = ImagesOverhead()
root.mapHistory = MapHistory()
#root.iui = handlers.StaticFiles(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'iui'))
root.angular = handlers.StaticFiles(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'angular'))
root.js = handlers.StaticFiles(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'js'))

