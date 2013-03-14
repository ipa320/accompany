import sys
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
		try:
			data, _ = self.sock.recvfrom(10240, 0)
		except Exception as e:
			if type(e) != timeout:
				print e
			return

		(_, mac, channel, val) = data.split(' ')

		mac = mac.lower()
		channel = channel.upper()
		
		try:
			sensor = next(s for s in self._sensors if s['ChannelDescriptor'] == str(mac) + str(channel))
		except StopIteration:
			#Only warn once, or we'll flood the console
			if str(mac) + str(channel) not in self._warned:
				print >> sys.stderr, "Warning: Unable to locate sensor record for zigbee sensor ID: %s" % (str(mac) + str(channel))
				self._warned.append(str(mac) + str(channel))
			return

		_device = sensor['locationName']
		_pin = sensor['name']
		_id = sensor['sensorId']

		if val != '-' and val != '':
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
		rows = self._geoDao.getData(self._geoQuery)
		#This appears to be needed or 'CALL exppower' doesn't update the power values,
		# oddly, it updates the timestamp field though...
		self._geoDao.close()
		for row in rows:
			try:
				sensor = next(s for s in self._sensors if s['ChannelDescriptor'] == str(row['ID']))
			except StopIteration:
				#Only warn once, or we'll flood the console
				if row['ID'] not in self._warned:
					print >> sys.stderr, "Warning: Unable to locate sensor record for geo sensor %s. ID: %s" % (row['Description'], row['ID'])
					self._warned.append(row['ID'])
				continue

			_device = sensor['locationName']
			_name = sensor['name']
			_id = sensor['sensorId']

			#Only warn once, or we'll flood the console
			if _name != row['Description'] and row['ID'] not in self._warned:
				print >> sys.stderr, 'Warning: Channel name differs from Geo-System description: %s / %s' % (_name, row['Description'])
				self._warned.append(row['ID'])

			_state = self._sr.evaluateRule(sensor['sensorRule'], row['Power'])

			if _state:
				_state = 'On'
			else:
				_state = 'Off'

			self._channels[row['ID']] = { 
										'id': _id, 
										'room': _device, 
										'channel': _name, 
										'value': '%.1f' % row['Power'], 
										'status': _state 
										}

if __name__ == '__main__':
	from config import server_config
	z = ZigBee(server_config['udp_listen_port'])
	z.start()

	g = GEOSystem(server_config['mysql_geo_server'],
                            server_config['mysql_geo_user'],
                            server_config['mysql_geo_password'],
                            server_config['mysql_geo_db'],
                            server_config['mysql_geo_query'])
	g.start()

	while True:
		try:
			sys.stdin.read()
		except KeyboardInterrupt:
			break

	z.stop()
	g.stop()