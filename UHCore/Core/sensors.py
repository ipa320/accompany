#!/usr/bin/env python
import sys
from socket import AF_INET, SOCK_DGRAM, socket, timeout
from Data.dataAccess import SQLDao, Sensors, Locations
from Data.stateResolver import StateResolver
import json, urllib2, base64

from extensions import PollingProcessor

""" Implementations of various sensor network updaters """

class _valueHelper(object):
	@staticmethod
	def filterValue(value, sensorType):
		""" Used by all sensor processors to pre-process raw values from various sensors """
		val = value
		if value == '-' or value == '':
			val = None
		elif sensorType == 'TEMPERATURE_MCP9700_HOT' or sensorType == 'TEMPERATURE_MCP9700_COLD':
			val = str((float(value) - 0.5) * 100.0)
		elif sensorType == 'LEVEL_SENSOR':
			val = int((value - 512) / 100)
			if val < 0:
				val = 0
		elif sensorType == 'POWER_CONSUMPTION_MONITOR':
			try:
				val = '%.1f' % float(val)
			except Exception as e:
				print "Err: %s" % val
		elif value == True: #1 and 0 are preferred for value
			val = 1
		elif value == False:
			val = 0
		return val

################################################################################
#
# ZWave thread
#
# Listens to the ZWave gateway's http api, transforms the
# channel values according to the specified sensor kind and puts them in
# the channel array.
#
################################################################################
class ZWaveHomeController(PollingProcessor):

	# Initialisation method
	# Opens the configured UDP socket for the receipt of broadcast messages.
	def __init__ (self, ipAddress):
		super(ZWaveHomeController, self).__init__()
		
		self._baseUrl = "http://%(ip)s:80/api/devices" % {'ip': ipAddress}

		# Place for the callback methods to store "static" values.
		# Currently only needed for the Moving Average Filter for the MCP9700
		# temperature sensor temperature calculations.
		self._handler_memory = {}
		self._channels = {}
		self._sr = StateResolver()
		self._sensorDao = Sensors()
		self._sensors = self._sensorDao.findSensors()
		self._warned = []

	@property
	def channels(self):
		if self._channels == None:
			self._channels = {}
		
		return self._channels

	def start(self):
		print "Started polling zwave sensors"
		self._addPollingProcessor('zwave', self.pollZWaveSensors, None, 0.1)

	def stop(self):
		print "Stopped polling zwave sensors"
		self._removePollingProcessor('zwave')
		
	def pollZWaveSensors(self):
		""" Check the http api for sensors & values and add them to the channels list in the standard format """
		try:
			# http://192.168.1.109/devices
			url = self._baseUrl
			request = urllib2.Request(url)
			base64string = base64.encodestring('%s:%s' % ('admin', 'admin')).replace('\n', '')
			request.add_header("Authorization", "Basic %s" % base64string)
			result = urllib2.urlopen(request)
			data = json.load(result) 
		except Exception as e:
			if str(type(e)) not in self._warned:
				print >> sys.stderr, "Error while receiving data from ZWaveHomeController: %s" % e
				self._warned.append(str(type(e)))
			return
		
		for device in data:
			channelDescriptor = 'zwave:' + str(device['id'])
			try:
				sensor = next(s for s in self._sensors if s['ChannelDescriptor'] == channelDescriptor)
			except StopIteration:
				# Only warn once, or we'll flood the console
				if channelDescriptor not in self._warned:
					print "Warning: Unable to locate sensor record for ZWave sensor ID: %s (%s)" % (str(channelDescriptor), str(device['name']))
					self._warned.append(channelDescriptor)
				continue
	
			_device = sensor['locationName']
			_pin = sensor['name']
			_id = sensor['sensorId']
			_rule = sensor['sensorRule']
	
			# order determines priority
			valueKeys = ['valueSensor', 'value']
			
			_value = None
			for valueKey in valueKeys:
				if device['properties'].has_key(valueKey):
					_value = device['properties'][valueKey]
					break
				
			_value = _valueHelper.filterValue(_value, sensor['sensorTypeName'])
			if _value != None:
				_type = sensor['sensorTypeName']
				_uuid = channelDescriptor
				_status = self._sr.getDisplayState({'sensorTypeName': _type, 'value': _value, 'sensorId': _id, 'sensorRule': _rule })
	
				self._channels[_uuid] = { 
										'id': _id,
										'room': _device,
										'channel': _pin,
										'value': _value,
										'status': _status
										}


################################################################################
#
# ZWave thread
#
# Listens to the ZWave gateway's http api, transforms the
# channel values according to the specified sensor kind and puts them in
# the channel array.
#
################################################################################
class ZWaveVeraLite(PollingProcessor):

	# Initialisation method
	# Opens the configured UDP socket for the receipt of broadcast messages.
	def __init__ (self, ipAddress, port):
		super(ZWaveVeraLite, self).__init__()
		
		self._loadTime = None
		self._dataVersion = None
		self._baseUrl = "http://%(ip)s:%(port)s/data_request?id=lu_sdata&timeout=%(timeout)s" % {'ip': ipAddress, 'port': port, 'timeout': 60}

		# Place for the callback methods to store "static" values.
		# Currently only needed for the Moving Average Filter for the MCP9700
		# temperature sensor temperature calculations.
		self._handler_memory = {}
		self._channels = {}
		self._sr = StateResolver()
		self._sensorDao = Sensors()
		self._sensors = self._sensorDao.findSensors()
		self._warned = []

	@property
	def channels(self):
		if self._channels == None:
			self._channels = {}
		
		return self._channels

	def start(self):
		print "Started polling zwave sensors"
		self._addPollingProcessor('zwave', self.pollZWaveSensors, None, 0.1)

	def stop(self):
		print "Stopped polling zwave sensors"
		self._removePollingProcessor('zwave')
		
	# ZigBee thread main loop
	def pollZWaveSensors(self):
		""" Check the http api for sensors & values and add them to the channels list in the standard format """
		""" first iteration pulls all data, subsequent loops pull only changed data since the last run """
		try:
			# http://192.168.1.158:3480/data_request?id=lu_sdata
			# http://192.168.1.158:3480/data_request?id=lu_sdata&loadtime=1282441735&dataversion=441736333&timeout=60
			url = self._baseUrl
			if self._loadTime != None and self._dataVersion != None:
				url += '&loadtime=%(load)s&dataversion=(dataversion)s' % {'load': self._loadTime, 'dataversion': self._dataVersion }
			data = json.load(urllib2.urlopen(url)) 
		except Exception as e:
			if str(type(e)) not in self._warned:
				print >> sys.stderr, "Error while receiving data from ZWaveVeraLite: %s" % e
				self._warned.append(str(type(e)))
			return
		
		self._loadTime = data['loadtime']
		self._dataVersion = data['dataversion']
		
		for device in data['devices']:
			channelDescriptor = 'zwave:' + str(device['id'])
			try:
				sensor = next(s for s in self._sensors if s['ChannelDescriptor'] == channelDescriptor)
			except StopIteration:
				# Only warn once, or we'll flood the console
				if channelDescriptor not in self._warned:
					print "Warning: Unable to locate sensor record for ZWave sensor ID: %s (%s)" % (str(channelDescriptor), str(device['name']))
					self._warned.append(channelDescriptor)
				continue
	
			_device = sensor['locationName']
			_pin = sensor['name']
			_id = sensor['sensorId']
			_rule = sensor['sensorRule']

			valueKeys = ['armed', 'status']

			_value = None
			for valueKey in valueKeys:
				if device.has_key(valueKey):
					_value = device[valueKey]
					break
	
			_value = _valueHelper.filterValue(_value, sensor['sensorTypeName'])
			if _value != None:
				_type = sensor['sensorTypeName']
				_uuid = channelDescriptor
				_status = self._sr.getDisplayState({'sensorTypeName': _type, 'value': _value, 'sensorId': _id, 'sensorRule': _rule })
	
				self._channels[_uuid] = { 
										'id': _id,
										'room': _device,
										'channel': _pin,
										'value': _value,
										'status': _status
										}
				
class ZigBeeDirect(PollingProcessor):
	
	def __init__(self, usbPort, baudRate=9600):
		super(ZigBeeDirect, self).__init__()
		import serial
		from Lib.xbee import ZigBee
		try:
			self._port = serial.Serial(usbPort, baudRate)
		except Exception as e:
			print >> sys.stderr, "Unable to connect to zigbee port, check that the port name (%s) and permissions are correct (should be a+rw)" % (usbPort)
			raise e
		self._zigbee = ZigBee(self._port)
		
		# Place for the callback methods to store "static" values.
		# Currently only needed for the Moving Average Filter for the MCP9700
		# temperature sensor temperature calculations.
		self._handler_memory = {}
		self._channels = {}
		self._sr = StateResolver()
		self._sensorDao = Sensors()
		self._sensors = self._sensorDao.findSensors()
		self._warned = []
	
	def __del__(self):
		self._port.close()
		super(ZigBeeDirect, self).__del__()
	
	@property
	def channels(self):
		if self._channels == None:
			self._channels = {}
		
		return self._channels
	
	def start(self):
		print "Started polling directly connected zigBee sensors"
		self._addPollingProcessor('zigBeeDirect', self.pollZigbeeSensors, None, 0.1)

	def stop(self):
		print "Stopped polling directly connected zigBee sensors"
		self._removePollingProcessor('zigBeeDirect')
		
	# ZigBee thread main loop
	def pollZigbeeSensors(self):
		""" Read the data from the Zigbee sensors directly connected to this machine """
		try:
			#data, _ = self._xbee.wait_read_frame()
			data = self._zigbee.wait_read_frame()
		except Exception as e:
			if str(type(e)) not in self._warned:
				print >> sys.stderr, "Error while receiving data from ZigBeeDirect: %s" % e
				self._warned.append(str(type(e)))
			return

		if data["id"] == "rx_explicit":
			
			mac = repr(data['source_addr_long']).replace('\\x', ':').strip(":'").lower()
			
			# If NI (Network Id)recognised include NI string in returned values
			try:
				channels = self._zigbee._parse_samples(data['rf_data'])[0] # Parse IO data
			except Exception as e:
				print >> sys.stderr, "Error reading zigbee data: %s" % e
				return
			
			for channel, _value in channels.items():
				channel = "!" + channel.lower()

				try:
					sensor = next(s for s in self._sensors if s['ChannelDescriptor'] == str(mac) + str(channel))
				except StopIteration:
					# Only warn once, or we'll flood the console
					if str(mac) + str(channel) not in self._warned:
						print "Warning: Unable to locate sensor record for ZigBee sensor ID: %s" % (str(mac) + str(channel))
						self._warned.append(str(mac) + str(channel))
					continue
					
				_device = sensor['locationName']
				_pin = sensor['name']
				_id = sensor['sensorId']
				_rule = sensor['sensorRule']
		
				_value = _valueHelper.filterValue(_value, sensor['sensorTypeName'])
				if _value != None:
					_type = sensor['sensorTypeName']
					_uuid = '%s_%s' % (mac , channel)
					_status = self._sr.getDisplayState({'sensorTypeName': _type, 'value': _value, 'sensorId': _id, 'sensorRule': _rule })
		
					self._channels[_uuid] = { 
											'id': _id,
											'room': _device,
											'channel': _pin,
											'value': _value,
											'status': _status
											}

################################################################################
#
# ZigBee thread
#
# Listens to the ZigBee gateway's UDP broadcast messages, transforms the
# channel values according to the specified sensor kind and puts them in
# the channel array.
#
################################################################################
class ZigBee(PollingProcessor):

	# Initialisation method
	# Opens the configured UDP socket for the receipt of broadcast messages.
	def __init__ (self, udpPort):
		super(ZigBee, self).__init__()
		self.sock = socket(AF_INET, SOCK_DGRAM)
		self.sock.settimeout(1)
		self.sock.bind(('', udpPort))

		# Place for the callback methods to store "static" values.
		# Currently only needed for the Moving Average Filter for the MCP9700
		# temperature sensor temperature calculations.
		self._handler_memory = {}
		self._channels = {}
		self._sr = StateResolver()
		self._sensorDao = Sensors()
		self._sensors = self._sensorDao.findSensors()
		self._warned = []

	@property
	def channels(self):
		if self._channels == None:
			self._channels = {}
		
		return self._channels

	def start(self):
		print "Started polling zigBee sensors"
		self._addPollingProcessor('zigBee', self.pollZigbeeSensors, None, 0.0001)

	def stop(self):
		print "Stopped polling zigBee sensors"
		self._removePollingProcessor('zigBee')
		
	# ZigBee thread main loop
	def pollZigbeeSensors(self):
		""" Check the read sensors & values from the zigbee UDP broadcast and add them to the channels list in the standard format """
		try:
			data, _ = self.sock.recvfrom(10240, 0)
		except Exception as e:
			if type(e) != timeout:
				print e
			return

		(_, mac, channel, _value) = data.split(' ')

		mac = mac.lower()
		channel = channel.upper()
		
		try:
			sensor = next(s for s in self._sensors if s['ChannelDescriptor'] == str(mac) + str(channel))
		except StopIteration:
			# Only warn once, or we'll flood the console
			if str(mac) + str(channel) not in self._warned:
				print "Warning: Unable to locate sensor record for ZigBee sensor ID: %s" % (str(mac) + str(channel))
				self._warned.append(str(mac) + str(channel))
			return

		_device = sensor['locationName']
		_pin = sensor['name']
		_id = sensor['sensorId']
		_rule = sensor['sensorRule']

		_value = _valueHelper.filterValue(_value, sensor['sensorTypeName'])
		if _value != None:
			_type = sensor['sensorTypeName']
			_uuid = '%s_%s' % (mac , channel)
			_status = self._sr.getDisplayState({'sensorTypeName': _type, 'value': _value, 'sensorId': _id, 'sensorRule': _rule })

			self._channels[_uuid] = { 
									'id': _id,
									'room': _device,
									'channel': _pin,
									'value': _value,
									'status': _status
									}

################################################################################
#
# MySQL thread
#
# Polls the Geo-System MySQL server once every second, transforms the data
# accordingly and stores them into the channel array.
#
################################################################################
class GEOSystem(PollingProcessor):
	
	def __init__ (self, hostName, userName, password, database, query):
		super(GEOSystem, self).__init__()
		self._geoDao = SQLDao(hostName, userName, password, database)		
		self._geoQuery = query
		self._sensorDao = Sensors()
		self._sensors = self._sensorDao.findSensors()
		self._sr = StateResolver()
		self._channels = {}
		self._warned = []

	@property
	def channels(self):		
		return self._channels

	def start(self):
		print "Started polling geo sensors"
		self._addPollingProcessor('geoSensors', self.pollGeoSystem, None, 0.1)

	def stop(self):
		print "Stopped polling geo sensors"
		self._removePollingProcessor('geoSensors')

	def pollGeoSystem(self):
		""" Check the GEO System database for sensors & values and add them to the channels list in the standard format """
		rows = self._geoDao.getData(self._geoQuery)
		# This appears to be needed or 'CALL exppower' doesn't update the power values,
		# oddly, it updates the timestamp field though...
		self._geoDao.close()
		for row in rows:
			try:
				sensor = next(s for s in self._sensors if s['ChannelDescriptor'] == str(row['ID']))
			except StopIteration:
				# Only warn once, or we'll flood the console
				if row['ID'] not in self._warned:
					print "Warning: Unable to locate sensor record for GEO sensor %s. ID: %s" % (row['Description'], row['ID'])
					self._warned.append(row['ID'])
				continue

			_device = sensor['locationName']
			_name = sensor['name']
			_id = sensor['sensorId']
			_type = sensor['sensorTypeName']
			_rule = sensor['sensorRule']
			_value = _valueHelper.filterValue(row['Power'], sensor['sensorTypeName'])

			# Only warn once, or we'll flood the console
			if _name != row['Description'] and row['ID'] not in self._warned:
				print 'Warning: Channel name differs from GEO-System description: %s / %s' % (_name, row['Description'])
				self._warned.append(row['ID'])

			_status = self._sr.getDisplayState({'sensorTypeName': _type, 'value': _value, 'sensorId': _id, 'sensorRule': _rule })

			self._channels[row['ID']] = { 
										'id': _id,
										'room': _device,
										'channel': _name,
										'value': _value,
										'status': _status
										}

if __name__ == '__main__':
	""" Run sensor updaters for all sensor types configured in the locations_config for the currently """
	""" active experimental location """
	import config
	from history import SensorLog
	
	activeLocation = Locations().getActiveExperimentLocation() 
	
	if activeLocation == None:
		print "Unable to determine active experiment Location"
		exit
	
	sensorPollers = []
	dataUpdaters = []
	for sensorType in config.locations_config[activeLocation['location']]['sensors']:
		sensor = None
		if sensorType == 'ZWaveHomeController':
			sensor = ZWaveHomeController(config.server_config['zwave_ip'])
		elif sensorType == 'ZWaveVeraLite':
			sensor = ZWaveVeraLite(config.server_config['zwave_ip'], config.server_config['zwave_port'])
		elif sensorType == 'ZigBee':
			sensor = ZigBee(config.server_config['udp_listen_port'])
		elif sensorType == 'ZigBeeDirect':
			sensor = ZigBeeDirect(config.server_config['zigbee_usb_port'])
		elif sensorType == 'GEOSystem':
			sensor = GEOSystem(config.server_config['mysql_geo_server'],
		                    config.server_config['mysql_geo_user'],
		                    config.server_config['mysql_geo_password'],
		                    config.server_config['mysql_geo_db'],
		                    config.server_config['mysql_geo_query'])
	
		if sensor != None:
			sensorPollers.append(sensor)
			dataUpdaters.append(SensorLog(sensor.channels, sensor.__class__.__name__))

	for sensorPoller in sensorPollers:
		sensorPoller.start()
	
	for dataUpdater in dataUpdaters:
		dataUpdater.start()
	
	while True:
		try:
			sys.stdin.read()
		except KeyboardInterrupt:
			break
	
	for sensorPoller in sensorPollers:
		sensorPoller.stop()
	
	for dataUpdater in dataUpdaters:
		dataUpdater.stop()
