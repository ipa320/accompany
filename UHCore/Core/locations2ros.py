#!/usr/bin/env python
import math

from Data.dataAccess import Sensors, Locations
from Robots.rosHelper import ROS
from extensions import PollingProcessor


class ROSLocationProcessor(PollingProcessor):
    """Abstract location processor, current concrete implementations are Human and Robot processors"""

    def __init__(self):
        super(ROSLocationProcessor, self).__init__()
        self._lastSensors = {}
        self._lastLocations = {}
        self._sensors = Sensors()
        self._locations = Locations()
        self._ros = ROS()

    def start(self):
        print "Started polling uh locations"
        self._addPollingProcessor('location_publish', self.checkUpdateLocations, (), 0.1)
        self._addPollingProcessor('sensor_publish', self.checkUpdateSensors, (), 0.1)

    def stop(self):
        print "Stopped polling uh locations"
        self._addPollingProcessor('location_publish')
        self._addPollingProcessor('sensor_publish')

    def checkUpdateLocations(self):
        locationData = self._locations.findLocations()
        locations = {}
        for location in locationData:
            locations[location['locationId']] = {
                'id': location['locationId'],
                'text': location['name'] or '',
                'position': {'x': location['xCoord'] or 0, 'y': location['yCoord'] or 0},
                'orientation': {'theta': math.radians(location['orientation'] or 0)},
                'color': {'r': float(not location['ValidRobotLocation'] and not location['ValidUserLocation']),
                          'g': float(location['ValidRobotLocation'] or 0),
                          'b': float(location['ValidUserLocation'] or 0),
                          'a': 1.0}
            }

        if self._lastLocations != locations:
            self._lastLocations = locations
            self._ros.setParam('location_data', locations.values())

    def checkUpdateSensors(self):
        sensorData = self._sensors.findSensors()
        sensors = {}
        for sensor in sensorData:
            sensors[sensor['sensorId']] = {
                'id': sensor['locationId'],
                'text': sensor['name'] or '',
                'position': {'x': sensor['xCoord'] or 0, 'y': sensor['yCoord'] or 0},
                'orientation': {'theta': math.radians(sensor['orientation'] or 0)},
                'color': {'r': 0.25,
                          'g': 0.25,
                          'b': 0.25,
                          'a': 1.0}
            }
            if sensor['isActive']:
                sensors[sensor['sensorId']]['color']['r'] = 0
                sensors[sensor['sensorId']]['color']['g'] = 1.0
                sensors[sensor['sensorId']]['color']['b'] = 0

            if not sensor['xCoord'] and not sensor['yCoord']:
                sensors[sensor['sensorId']]['position']['z'] = -1

        if self._lastSensors != sensors:
            self._lastSensors = sensors
            self._ros.setParam('sensor_data', sensors.values())


if __name__ == '__main__':
    import sys
    lp = ROSLocationProcessor()
    lp.start()
    while True:
        try:
            sys.stdin.read()
        except KeyboardInterrupt:
            break
    lp.stop()
