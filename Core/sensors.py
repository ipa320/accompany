from socket import AF_INET, SOCK_DGRAM, socket, timeout
from Data.dataAccess import SQLDao, Sensors
from Data.sensors import StateResolver

from extensions import PollingProcessor

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

	@property
	def channels(self):
		if self._channels == None:
			self._channels = {}
		
		return self._channels

	def start(self):
		print "Started polling zigBee sensors"
		self._addPollingProcessor('zigBee', self.pollZigbeeSensors, None, 0.1)

	def stop(self):
		print "Stopped polling zigBee sensors"
		self._removePollingProcessor('zigBee')
		
	# ZigBee thread main loop
	def pollZigbeeSensors(self):
		try:
			data, _ = self.sock.recvfrom(10240, 0)
		except Exception as e:
			if type(e) != timeout:
				print e
			return
		
		(_, mac, channel, val) = data.split(' ')

		mac = mac.lower()
		channel = channel.upper()
		
		sensor = next(s for s in self._sensors if s['ChannelDescriptor'] == str(mac) + str(channel))
		try:
			#_device = zigbee_devices[mac]['room']
			_device = sensor['locationName']
		#except:
		#	return

		#try:
			#_pin = zigbee_devices[mac][channel]['name']
			#_id = zigbee_devices[mac][channel]['id']
			_pin = sensor['name']
			_id = sensor['sensorId']
		except:
			return

		if val != '-' and val != '':
			#_type = zigbee_devices[mac][channel]['type']
			_type = sensor['sensorTypeName']
			_uuid = '%s_%s' % (mac , channel)
			if _type == 'TEMPERATURE_MCP9700_HOT' or _type == 'TEMPERATURE_MCP9700_COLD':
				_value = str((float( val) - 0.5) * 100.0)
			else:
				_value = val
			
			_status = self._sr.getDisplayState({'sensorTypeName': _type, 'value': _value, 'sensorId': _id })

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
		rows = self._geoDao.getData(self._geoQuery)
		for row in rows:
			sensor = next(s for s in self._sensors if s['ChannelDescriptor'] == row['ID'])
			try:
				#_device = geosystem_devices[row['ID']]['room']
				#_name = geosystem_devices[row['ID']]['name']
				#_id = geosystem_devices[row['ID']]['id']
				_device = sensor['locationName']
				_name = sensor['name']
				_id = sensor['sensorId']
			except:
				continue

			if _name != row['Description']:
				print 'Warning: Channel name differs from Geo-System description: %s / %s' % (_name, row['Description'])

			#p is used in the eval rule
			#p = row['Power']
			#_state = eval(geosystem_devices[row['ID']]['rule'])
			#if _state:
			#	_state = 'On'
			#else:
			#	_state = 'Off'
			_state = self._sr.evaluateRule(sensor['rule'], row['Power'])

			self._channels[row['ID']] = { 
										'id': _id, 
										'room': _device, 
										'channel': _name, 
										'value': '%.1f' % row['Power'], 
										'status': _state 
										}

if __name__ == '__main__':
	from config import server_config
	import sys
	z = ZigBee(server_config['udp_listen_port'])
	z.start()
	
	g = GEOSystem(server_config['mysql_geo_server'],
                            server_config['mysql_geo_user'],
                            server_config['mysql_geo_password'],
                            server_config['mysql_geo_db'],
                            server_config['mysql_geo_query'])
	#g.start()
	
	while True:
		try:
			sys.stdin.read()
		except KeyboardInterrupt:
			break

	z.stop()
	g.stop()