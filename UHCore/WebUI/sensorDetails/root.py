import handlers, os
import states

name = "Live Sensor Details"
__dir = os.path.dirname(os.path.realpath(__file__))

root = handlers.Index(os.path.join(__dir, 'sensorstates.html'))
root.images = handlers.StaticFiles(os.path.join(__dir, 'images'))
root.js = handlers.StaticFiles(os.path.join(__dir, 'js'))
root.css = handlers.StaticFiles(os.path.join(__dir, 'css'))
root.data = states.States()