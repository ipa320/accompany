import sys
import dataAccess

class StateResolver(object):

    def __init__(self):
        self._dao = dataAccess.DataAccess()
        self._movingAverageCache = {}
        self._movingAverageCacheLength = 20
    
    def isSensorOn(self, sensor):
        rule = sensor['sensorRule']
        value = sensor['value']
        if rule == 'Moving Average':
            return self.temperatureStatus(sensor).lower() == 'on'
        elif rule == 'Boolean':
            return self.evaluateBoolean(sensor['sensorTypeName'], value)
        elif rule.find('Watts') > -1:
            return self.evaluateRule(rule, value)
        else:
            return None
        
    def evaluateBoolean(self, sensorType, value):            
        if sensorType == 'CONTACT_REED':
            return float(value) == 1
        elif sensorType == 'CONTACT_PRESSUREMAT':
            return float(value) != 1
        else:
            return None
    
    def evaluateRule(self, rule, value):
            try:
                #Rules are in Java notation
                pyRule = rule.replace('&&', 'and').replace('||', 'or')
                #Watts = float(value)
                pyRule = pyRule.replace('Watts', str(value))
                return eval(pyRule)
            except Exception as e:
                print >> sys.stderr, 'Error parsing rule "%(rule)s", Exception: %(exception)s' % { 'rule': rule, 'exception': e }
                return None
    
    def resolveStates(self, sensorList):
        """returns [{'id': sensor['sensorId'], 'value': sensor['value'], 'state':'Open'/'', 'on':True/False/None},]"""
        states = []
                
        for sensor in sensorList:
            state = {
                     'id': sensor['sensorId'], 
                     'value': sensor['value'],
                     'on': self.isSensorOn(sensor),
                     'state': self.getDisplayState(sensor)}
            
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
    
    def getDisplayState(self, sensor):
        stype = sensor['sensorTypeName']
        if stype == 'CONTACT_REED':
            if float(sensor['value']) == 1:
                return 'Open'
            else:
                return 'Closed'
        elif stype == 'CONTACT_PRESSUREMAT':
            if float(sensor['value']) == 1:
                return 'Free'
            else:
                return 'Occupied'
        elif stype == 'TEMPERATURE_MCP9700_HOT' or stype == 'TEMPERATURE_MCP9700_COLD':
            #return str((float(sensor['value']) - 0.5) * 100.0) + 'C'
            return self.temperatureStatus(sensor)
        elif stype == 'POWER_CONSUMPTION_MONITOR':
            if float(sensor['value']) > 0.1:
                return 'On'
            else:
                return 'Off'
        else:
            return str(sensor['value'])

    def temperatureStatus(self, sensor):
        if not self._movingAverageCache.has_key(sensor['sensorId']):
            self._movingAverageCache[sensor['sensorId']] = { 'values':[], 'status': False }
            
        valmem = self._movingAverageCache[sensor['sensorId']]['values']
        status = self._movingAverageCache[sensor['sensorId']]['status']

        temp = (float(sensor['value']) - 0.5) * 100.0
        valmem.append(temp)
        
        if len(valmem) > self._movingAverageCacheLength:
            valmem.pop(0)

        avg = sum(valmem) / len(valmem)
        
        if sensor['sensorTypeName'] == 'TEMPERATURE_MCP9700_HOT':
            if (status == False and temp >= 1.1 * avg):
                status = True
            elif (status == True and temp <= 0.9 * avg):
                status = False
        elif sensor['sensorTypeName'] == 'TEMPERATURE_MCP9700_COLD':
            if (status == False and temp <= 0.9 * avg):
                status = True
            elif (status == True and temp >= 1.1 * avg):
                status = False
        else:
            print >> sys.stderr, 'Unknown moving average sensor type: %s' % (sensor['sensorTypeName'])
            status = None
            
        if status:
            return 'On'
        else:
            return 'Off'

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