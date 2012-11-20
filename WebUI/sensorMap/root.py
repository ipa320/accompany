import handlers, os
import currentMap

name = "Live Sensor View"
__dir = os.path.dirname(os.path.realpath(__file__))

root = handlers.Index(os.path.join(__dir, 'sensormap.html'))
root.image = currentMap.MapImage()
root.js = handlers.StaticFiles(os.path.join(__dir, 'js'))
root.css = handlers.StaticFiles(os.path.join(__dir, 'css'))
root.details = handlers.Index(os.path.join(__dir, 'sensorstates.html'))