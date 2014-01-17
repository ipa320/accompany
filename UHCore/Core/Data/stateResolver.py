import sys
import dataAccess

class StateResolver(object):
    """ Used to resolve the on/off state for a given sensor"""
    def __init__(self):
        self._dao = dataAccess.Sensors()
        self._movingAverageCache = {}
        self._sensorTypeCache = {}
        self._movingAverageCacheLength = 20
        self._warned = {}
    
    def isSensorOn(self, sensor):
        """ Evaluate a sensors rules into a boolean value """
        rule = sensor['sensorRule']
        value = sensor['value']
        if rule == 'Moving Average':
            return self.temperatureStatus(sensor).lower() == 'on'
        elif rule == 'Boolean':
            return self.evaluateBoolean(sensor['sensorTypeName'], value)
        elif rule == '!Boolean':
            return not self.evaluateBoolean(sensor['sensorTypeName'], value)
        elif rule.find('Watts') > -1:
            return self.evaluateRule(rule, value)
        elif rule == 'N/A':
            return None
        elif rule == 'Level':
            return self.evaluateLevel(value)
        else:
            print >> sys.stderr, "Unknown sensor rule: %s" % rule
            return None
        
    def evaluateLevel(self, value):
        """ Simple evaluation for level sensors """
        return value > 1
        
    def evaluateBoolean(self, sensorType, value):
        """ Evaluation for boolean sensor types (REED, PRESSUREMAP, SWITCH) """
        if sensorType == 'CONTACT_REED':
            return float(value) == 1
        elif sensorType == 'CONTACT_PRESSUREMAT':
            return float(value) != 1
        elif sensorType == 'LEVEL_SENSOR_SWITCH':
            return float(value) == 1
        else:
            if not self._warned.has_key(sensorType):
                print "Unknown boolean sensorType: %s, defaulting to 0 == False all else True" % sensorType
                self._warned[sensorType] = True
            try:
                return float(value) != 0
            except:
                return True
    
    def evaluateRule(self, rule, value):
        """ Evaluation for rule based sensors (Currently use only for POWERLINE sensors) """
        try:
            # Rules are in Java notation
            pyRule = rule.replace('&&', 'and').replace('||', 'or')
            # Watts = float(value)
            pyRule = pyRule.replace('Watts', str(value))
            return eval(pyRule)
        except Exception as e:
            print >> sys.stderr, 'Error parsing rule "%(rule)s", Exception: %(exception)s' % { 'rule': rule, 'exception': e }
            return None
    
    def resolveStates(self, sensorList):
        """ Resolve the states of all sensors in the given list """
        """returns [{'id': sensor['sensorId'], 'value': sensor['value'], 'state':'Open/Closed/sensor['value']', 'on':True/False/None, 'xCoord': sensor['xCoord'], 'yCoord': sensor['yCoord'], 'orientation': sensor['orientation']}]"""
        states = []
                
        for sensor in sensorList:
            state = {
                      'id': sensor['sensorId'],
                      'value': sensor['value'],
                      'on': self.isSensorOn(sensor),
                      'state': self.getDisplayState(sensor),
                      'xCoord': sensor['xCoord'],
                      'yCoord': sensor['yCoord'],
                      'orientation': '%sd' % sensor['orientation'],
                      'icon': sensor['icon'],
                      }
             
            states.append(state)
            
        return states
        
    def _getType(self, typeName):
        """ Returns the type data for the given sensor """
        if not self._sensorTypeCache.has_key(typeName):
            self._sensorTypeCache[typeName] = self._dao.getSensorTypeByName(typeName)
        
        return self._sensorTypeCache[typeName]
    
    def getDisplayState(self, sensor):
        """ Gets the display value for the given sensor, given it's current state """
        stypename = sensor['sensorTypeName']
        stype = self._getType(stypename)
        
        isOn = self.isSensorOn(sensor) 
        if isOn:
            state = stype['active']
        elif isOn == False:
            state = stype['inactive']
        else: #isOn == None
            state = str(sensor['value'])
        
        return state.capitalize()        

    def temperatureStatus(self, sensor):
        """ Moving average-ish evaluation for temperature sensors """
        """ Temperature must change greater than 10% within the window for the sensor to be regarded as 'active' """
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
        """ No longer used, merged data from the old sensors xml file into the database """
        from dataAccess import DataAccess
        dao = DataAccess()
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
            
            dao.saveData(sql, args)

    def _alterMetaData(self):
        """ No longer used, converted SVG coords into RH coords """
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
