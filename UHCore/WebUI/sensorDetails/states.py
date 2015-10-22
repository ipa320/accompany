from Data.dataAccess import Sensors
from Data.stateResolver import StateResolver

from operator import itemgetter
import cherrypy
import json

class States(object):
    def __init__(self):
        self._dao = Sensors()
        self._sr = StateResolver()
    
    exposed = True
    
    def GET(self, *args, **kwargs):
        
        sensors = self._dao.findSensors()
        sensors = sorted(sensors, key = itemgetter('locationName', 'ChannelDescriptor'))
        
        elements = []
        for sensor in sensors:
            elements.append(
                            {
                             'color': self.getBackgroundColor(sensor),
                             'room': '%(loc)s: %(sen)s' % { 'loc': sensor['locationName'], 'sen': sensor['name'] },
                             'channel': sensor['ChannelDescriptor'],
                             'value': sensor['value'],
                             'status': self._sr.getDisplayState(sensor)
                             })
        
        cherrypy.response.headers['Content-Type'] = 'application/json'
        return json.dumps(elements)

    def getBackgroundColor(self, sensor):
        stype = sensor['sensorTypeName']
        if stype == 'CONTACT_REED':
            if float(sensor['value']) == 1:
                return '#FF0000'
            else:
                return '#00FF00' # closed door == green colour
        elif stype == 'CONTACT_PRESSUREMAT':
            if float(sensor['value']) == 1:
                return '#00FF00' # vacant chair == green colour
            else:
                return '#FF0000'
        elif stype == 'TEMPERATURE_MCP9700_HOT' or stype == 'TEMPERATURE_MCP9700_COLD':
            #return str((float(sensor['value']) - 0.5) * 100.0) + 'C'
            temp = float(sensor['value'])
            if temp < 0.0:
                r = 0
            elif temp > 50.0:
                r = 255
            else:
                r = int(temp * 5.1) # -0C..+50C -> 0..255
            g = 0
            b = 255 - r
            return '#%02X%02X%02X' % (r, g, b)
        elif stype == 'POWER_CONSUMPTION_MONITOR':
            if self._sr.isSensorOn(sensor):
                return '#FFFF00'
            else:
                return '#00FF00' # closed door == green colour
        else:
            return 'None'