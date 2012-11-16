from socket import AF_INET, SOCK_DGRAM, socket
from Data.dataAccess import SQLDao

from extensions import PollingProcessor
from config import zigbee_devices, geosystem_devices

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
		except:
			return
		
		ar = data.split(' ')

		try:
			_device = zigbee_devices[ar[1].lower()]['room']
		except:
			return

		try:
			_pin = zigbee_devices[ar[1].lower()][ar[2].upper()]['name']
			_id = zigbee_devices[ar[1].lower()][ar[2].upper()]['id']
		except:
			return

		if ar[3] != '-' and ar[3] != '':
			_type = zigbee_devices[ar[1].lower()][ar[2].upper()]['type']
			_uuid = 'ZigBee_' + ar[1].lower() + '_' + ar[2].upper()

			if _type == 'CONTACT_REED':
				_value = ar[3]
				_status = self.handler_contact_reed_status(_uuid, ar[3])
				#_color = self.handler_contact_reed_color(_uuid, ar[3])
			elif _type == 'CONTACT_PRESSUREMAT':
				_value = ar[3]
				_status = self.handler_contact_pressuremat_status(_uuid, ar[3])
				#_color = self.handler_contact_pressuremat_color(_uuid, ar[3])
			elif _type == 'TEMPERATURE_MCP9700_HOT':
				_value = self.handler_temperature_mcp9700_value(_uuid, ar[3])
				_status = self.handler_temperature_mcp9700_hot_status(_uuid, ar[3])
				#_color = self.handler_temperature_mcp9700_color(_uuid, ar[3])
			elif _type == 'TEMPERATURE_MCP9700_COLD':
				_value = self.handler_temperature_mcp9700_value(_uuid, ar[3])
				_status = self.handler_temperature_mcp9700_cold_status(_uuid, ar[3])
				#_color = self.handler_temperature_mcp9700_color(_uuid, ar[3])
			else:
				_value = ar[3]
				_status = 'undefined'
				#_color = '000000'

#			print 'ZigBee thread: %s %s: %s %s' % (_device, _pin, _value, _status)
			self._channels[_uuid] = { 'id': _id, 'room': _device, 'channel': _pin, 'value': _value, 'status': _status, 'color': _color }

################################################################################
#
# MySQL thread
#
# Polls the Geo-System MySQL server once every second, transforms the data
# accordingly and stores them into the channel array.
#
################################################################################
class GEOSystem(PollingProcessor):
	
	def __init__ (self):
		super(GEOSystem, self).__init__()
		self._sql = SQLDao(server_config['mysql_geo_server'],
							server_config['mysql_geo_user'],
							server_config['mysql_geo_password'],
							server_config['mysql_geo_db'])
		
		self._channels = None

	@property
	def channels(self):
		if self._channels == None:
			self._channels = {}
		
		return self._channels

	def start(self):
		print "Started polling geo sensors"
		self._addPollingProcessor('geoSensors', self.pollGeoSystem, None, 0.1)

	def stop(self):
		print "Stopped polling geo sensors"
		self._removePollingProcessor('geoSensors')

	def pollGeoSystem(self):
		rows = self._sql.getData(server_config['mysql_geo_query'])
		for row in rows:
			try:
				_device = geosystem_devices[row[0]]['room']
				_name = geosystem_devices[row[0]]['name']
				_id = geosystem_devices[row[0]]['id']
			except:
				continue

			if _name != row[1]:
				print 'Warning: Channel name differs from Geo-System description: %s / %s' % (_name, row[1])

			_state = eval(geosystem_devices[row[0]]['rule'])
			if _state:
				_state = 'On'
				#_color = '00FF00'
			else:
				_state = 'Off'
				#_color = 'FF0000'

#				print 'MySQL thread: %s: %s (%4.1f Watts)' % (_device, _state, row[1])
			self._channels['MySQL_' + str(row[0])] = { 'id': _id, 'room': _device, 'channel': _name, 'value': '%4.1f Watts' % row[2], 'status': _state } #, 'color': _color }

if __name__ == '__main__':
	from config import server_config
	import sys
	z = ZigBee()
	z.start()
	
	g = GEOSystem()
	g.start()
	
	while True:
		try:
			sys.stdin.read()
		except KeyboardInterrupt:
			break

	z.stop()
	g.stop()