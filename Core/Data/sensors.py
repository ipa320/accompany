import sys
import dataAccess

class StateResolver(object):

    def __init__(self):
        self._dao = dataAccess.DataAccess()
    
    def isSensorOn(self, sensor):
        rule = sensor['sensorRule']
        value = sensor['value']
        if rule == 'Moving Average':
            return sensor['value'] > 0
        elif rule == 'Boolean':
            return sensor['value'] == 1
        elif rule.find('Watts') > -1:
            try:
                #Rules are in Java notation
                pyRule = rule.replace('&&', 'and').replace('||', 'or')
                #Watts = float(value)
                pyRule = pyRule.replace('Watts', str(value))
                return eval(pyRule)
            except Exception as e:
                print >> sys.stderr, 'Error parsing rule "%(rule)s", Exception: %(exception)s' % { 'rule': rule, 'exception': e }
                return -1
        else:
            return -1
    
    def resolveStates(self, sensorList):
        """returns [{'id': sensor['sensorId'], 'value': sensor['value'], 'state':'on'},]"""
        states = []
                
        for sensor in sensorList:
            state = {'id': sensor['sensorId'], 'value': sensor['value']}
            
            ss = self.isSensorOn(sensor)
            if ss == 1:
                state['state'] = 'on'
            elif ss == 0:
                state['state'] = 'off'
            else:
                state['state'] = ''
            states.append(state)
            
        return states
    
    def appendSensorMetadata(self, sensorList):
        """adds location and type data to a list of sensors"""
        """this data eventually needs to go in the database"""
        from xml.etree import ElementTree as et
        from SensorMap.processor import CoordinateConvertor
        import os
        cc = CoordinateConvertor()
        sensors = et.parse(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'sensor_metadata.xml'))
        for sensor in sensorList:
            meta = None
            if sys.version_info >= (2,7):
                meta = sensors.getroot().find("./sensor[@id='%s']" % (sensor['id']))
            else:
                for s in sensors.getroot().findall('sensor'):
                    if long(s.attrib['id']) == sensor['id']:
                        meta = s
                        break
            if meta == None:
                sensor['location'] = cc.toRobotHouse((0,0,0))
                sensor['type'] = 'unknown'
                continue
            x = float(meta.get('x', 0))
            y = float(meta.get('y', 0))
            d = float(meta.get('direction', 0))
            sensor['location'] = (x, y, '%sd' % (d))
            sensor['type'] = meta.get('type', '')
        
        return sensorList    
    
    # Callback methods that transform the transmitted value into
    # the device's according status and generate a colour and a
    # human-readable value for the web-based presentation.
    def handler_contact_reed_status(self, channel_uuid, value):
        if value == '1':
            return 'Open'
        else:
            return 'Closed'

    def handler_contact_reed_color(self, channel_uuid, value):
        if value == '1':
            return 'FF0000'
        else:
            return '00FF00' # closed door == green colour

    def handler_contact_pressuremat_status(self, channel_uuid, value):
        if value == '1':
            return 'Free'
        else:
            return 'Occupied'

    def handler_contact_pressuremat_color(self, channel_uuid, value):
        if value == '1':
            return '00FF00' # vacant chair == green colour
        else:
            return 'FF0000'

    def handler_temperature_mcp9700_value(self, channel_uuid, value):
        return str((float(value) - 0.5) * 100.0) + 'C'

    def handler_temperature_mcp9700_hot_status(self, channel_uuid, value):
        channel_uuid = channel_uuid + '_handler_temperature_mcp9700_hot_status'
        filter_length = 20 # Moving Average Filter of length 20
        try:
            valmem = self.handler_memory[channel_uuid]['values']
            status = self.handler_memory[channel_uuid]['status']
        except:
            self.handler_memory[channel_uuid] = {}
            self.handler_memory[channel_uuid]['values'] = []
            valmem = self.handler_memory[channel_uuid]['values']
            self.handler_memory[channel_uuid]['status'] = False
            status = self.handler_memory[channel_uuid]['status']
        temp = (float(value) - 0.5) * 100.0
        valmem.append(temp)
        if len(valmem) > filter_length:
            valmem.pop(0)
        avg = sum(valmem) / len(valmem)
        if (status == False and temp >= 1.1 * avg):
            status = True
        elif (status == True and temp <= 0.9 * avg):
            status = False
        if status == True:
            return 'On'
        else:
            return 'Off'

    def handler_temperature_mcp9700_cold_status(self, channel_uuid, value):
        channel_uuid = channel_uuid + '_handler_temperature_mcp9700_cold_status'
        filter_length = 20 # Moving Average Filter of length 20
        try:
            valmem = self.handler_memory[channel_uuid]['values']
            status = self.handler_memory[channel_uuid]['status']
        except:
            self.handler_memory[channel_uuid] = {}
            self.handler_memory[channel_uuid]['values'] = []
            valmem = self.handler_memory[channel_uuid]['values']
            self.handler_memory[channel_uuid]['status'] = False
            status = self.handler_memory[channel_uuid]['status']
        temp = (float(value) - 0.5) * 100.0
        valmem.append(temp)
        if len(valmem) > filter_length:
            valmem.pop(0)
        avg = sum(valmem) / len(valmem)
        if (status == False and temp <= 0.9 * avg):
            status = True
        elif (status == True and temp >= 1.1 * avg):
            status = False
        if status == True:
            return 'On'
        else:
            return 'Off'

    def handler_temperature_mcp9700_color(self, channel_uuid, value):
        temp = (float(value) - 0.5) * 100.0
        if temp < 0.0:
            r = 0
        elif temp > 50.0:
            r = 255
        else:
            r = int(temp * 5.1) # -0C..+50C -> 0..255
        g = 0
        b = 255 - r
        return '%02X%02X%02X' % (r, g, b)
    
    def _importMetaData(self):
        from xml.etree import ElementTree as et
        import os
        sensors = et.parse(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'sensor_metadata.xml'))
        
        for sensor in sensors.getroot().findall("./sensor"):
            x = float(sensor.get('x', 0))
            y = float(sensor.get('y', 0))
            d = float(sensor.get('direction', 0))
            sid = int(sensor.get('id'))
            sql = "UPDATE `%s`" % (self._dao._sensorTable)
            sql += " SET `xCoord` = %(x)s, `yCoord` = %(y)s, `orientation` = %(d)s  WHERE `sensorId` = %(id)s" 
            args = {'x': x,
                    'y': y, 
                    'd': d, 
                    'id': sid}
            
            self._dao.saveData(sql, args)

    def _alterMetaData(self):
        from xml.etree import ElementTree as et
        from SensorMap.processor import CoordinateConvertor
        import os, math
        cc = CoordinateConvertor()
        sensors = et.parse(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'sensor_metadata.xml.bak'))
        for sensor in sensors.getroot().findall("./sensor"):
            x = float(sensor.get('x', 0))
            y = 728 - float(sensor.get('y', 0))
            d = float(sensor.get('direction', 0))
            (x, y, d) = cc.toRobotHouse((x, y, d))
            sensor.attrib['x'] = str(x)
            sensor.attrib['y'] = str(y)
            d = math.degrees(d)
            d = d % 360
            sensor.attrib['direction'] = str(d)
            
        sensors.write('sensor_metadata.xml')

if __name__ == '__main__':
    s = StateResolver()
    s._importMetaData()