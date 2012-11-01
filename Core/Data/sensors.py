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