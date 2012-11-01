import handlers, os
import currentMap

name = "Live Sensor View"

root = handlers.Index(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'sensormap.html'))
root.image = currentMap.MapImage()
root.js = handlers.StaticFiles(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'js'))
root.css = handlers.StaticFiles(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'css'))
