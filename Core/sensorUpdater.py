# -*- coding: utf-8 -*-
################################################################################
#
# Robot House Data Connector
#
# Main program file
#
# Last edited: 2010-12-14
# By:					Patrick Neuberger <p.neuberger@herts.ac.uk>
#
################################################################################

import socket
import SimpleHTTPServer
import SocketServer
import string, cgi, time
import time
import MySQLdb
import datetime

from threading import Thread
from socket import *
from operator import itemgetter, attrgetter
from os import curdir, sep
from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
from config import *

# For storing the dynamically recognized sensor channels and their values.
channels = {}

################################################################################
#
# ZigBee thread
#
# Listens to the ZigBee gateway's UDP broadcast messages, transforms the
# channel values according to the specified sensor kind and puts them in
# the channel array.
#
################################################################################

class ZigBeeThread(Thread):

	# Place for the callback methods to store "static" values.
	# Currently only needed for the Moving Average Filter for the MCP9700
	# temperature sensor temperature calculations.
	handler_memory = {}

	# Callback methods that transform the transmitted value into
	# the device's according status and generate a color and a
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
			return '00FF00' # closed door == green color

	def handler_contact_pressuremat_status(self, channel_uuid, value):
		if value == '1':
			return 'Free'
		else:
			return 'Occupied'

	def handler_contact_pressuremat_color(self, channel_uuid, value):
		if value == '1':
			return '00FF00' # vacant chair == green color
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

	# Initialization method
	# Opens the configured UDP socket for the receipt of broadcast messages.
	def __init__ (self):
		Thread.__init__(self)
		self.sock = socket(AF_INET, SOCK_DGRAM)
		self.sock.settimeout(1)
		self.sock.bind(('', server_config['udp_listen_port']))
		self.status = 1
		print 'ZigBee thread initialized'

	# ZigBee thread main loop
	def run(self):
		print 'ZigBee thread started'
		while self.status:
			try:
				data, addr = self.sock.recvfrom(10240, 0)
			except:
				continue
			ar = data.split(' ')

			try:
				_device = zigbee_devices[ar[1].lower()]['room']
			except:
				continue

			try:
				_pin = zigbee_devices[ar[1].lower()][ar[2].upper()]['name']
				_id = zigbee_devices[ar[1].lower()][ar[2].upper()]['id']
			except:
				continue

			if ar[3] != '-' and ar[3] != '':
				_type = zigbee_devices[ar[1].lower()][ar[2].upper()]['type']
				_uuid = 'ZigBee_' + ar[1].lower() + '_' + ar[2].upper()

				if _type == 'CONTACT_REED':
					_value = ar[3]
					_status = self.handler_contact_reed_status(_uuid, ar[3])
					_color = self.handler_contact_reed_color(_uuid, ar[3])
				elif _type == 'CONTACT_PRESSUREMAT':
					_value = ar[3]
					_status = self.handler_contact_pressuremat_status(_uuid, ar[3])
					_color = self.handler_contact_pressuremat_color(_uuid, ar[3])
				elif _type == 'TEMPERATURE_MCP9700_HOT':
					_value = self.handler_temperature_mcp9700_value(_uuid, ar[3])
					_status = self.handler_temperature_mcp9700_hot_status(_uuid, ar[3])
					_color = self.handler_temperature_mcp9700_color(_uuid, ar[3])
				elif _type == 'TEMPERATURE_MCP9700_COLD':
					_value = self.handler_temperature_mcp9700_value(_uuid, ar[3])
					_status = self.handler_temperature_mcp9700_cold_status(_uuid, ar[3])
					_color = self.handler_temperature_mcp9700_color(_uuid, ar[3])
				else:
					_value = ar[3]
					_status = 'undefined'
					_color = '000000'

#				print 'ZigBee thread: %s %s: %s %s' % (_device, _pin, _value, _status)
				channels[_uuid] = { 'id': _id, 'room': _device, 'channel': _pin, 'value': _value, 'status': _status, 'color': _color }

################################################################################
#
# MySQL thread
#
# Polls the Geo-System MySQL server once every second, transforms the data
# accordingly and stores them into the channel array.
#
################################################################################

class MySQLThread(Thread):

	# Initialization method
	# Does basically nothing
	def __init__ (self):
		Thread.__init__(self)
		self.status = 1
		print 'MySQL thread initialized'

	# MySQL thread main loop
	# Due to some problems in the past, the connection to the MySQL server
	# is now established and destroyed separately for each database access.
	def run(self):
		print 'MySQL thread started - joe skipped'

		while self.status:
			conn = MySQLdb.connect(server_config['mysql_geo_server'],
																	server_config['mysql_geo_user'],
																	server_config['mysql_geo_password'],
																	server_config['mysql_geo_db'])
			cursor = conn.cursor()
			cursor.execute(server_config['mysql_geo_query'])
			rows = cursor.fetchall()
			for row in rows:
				try:
					_device = geosystem_devices[row[0]]['room']
					_name = geosystem_devices[row[0]]['name']
					_id = geosystem_devices[row[0]]['id']
				except:
					continue

				if _name != row[1]:
					print 'Warning: Channel name differs from Geo-System description: %s / %s' % (_name, row[1])

				p = row[2]
				_state = eval(geosystem_devices[row[0]]['rule'])
				if _state:
					_state = 'On'
					_color = '00FF00'
				else:
					_state = 'Off'
					_color = 'FF0000'

#				print 'MySQL thread: %s: %s (%4.1f Watts)' % (_device, _state, row[1])
				channels['MySQL_' + str(row[0])] = { 'id': _id, 'room': _device, 'channel': _name, 'value': '%4.1f Watts' % row[2], 'status': _state, 'color': _color }

			cursor.close()
			conn.close()

			time.sleep(1)

################################################################################
#
# Logger thread
#
# Logs channel value and / or status changes into a (separate) MySQL
# database table.
#
################################################################################

class LoggerThread(Thread):

	# This method handles the actual writing procedure.
	def writechannel(self, channel):
		dateNow = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
		print dateNow

		cursor = self.conn.cursor()

		sql = "INSERT INTO `%s` (`timestamp`, `sensorId`, `room`, `channel`, `value`, `status`) VALUES ('%s', '%s', '%s', '%s', '%s', '%s')" % (
							server_config['mysql_log_table'], 
							dateNow, 
							channel['id'],
							channel['room'], 
							channel['channel'], 
							channel['value'], 
							channel['status'])
		try:
			cursor.execute(sql)
			self.conn.commit()
			print 'Logger thread: %s %s %s %s' % (channel['room'], channel['channel'], channel['value'], channel['status'])
			cursor.close()
			return True
		except MySQLdb.Error, e:
			print "Error %d: %s" % (e.args[0],e.args[1])

			self.conn.rollback()
			cursor.close()
			sys.exit(1)	
 
		
	# Initialization method
	# Establishes the connection to the MySQL database.
	def __init__ (self):
		Thread.__init__(self)
		self.conn = MySQLdb.connect(server_config['mysql_log_server'],
																server_config['mysql_log_user'],
																server_config['mysql_log_password'],
																server_config['mysql_log_db'])
		self.status = 1
		print 'Logger thread initialized'

	# Logger thread main loop
	def run(self):
		logged_channels = {}

		print 'Logger thread started'
		while self.status:
			key = channels.keys()
			for k in key:
				try:
#					if logged_channels[k]['value'] != channels[k]['value'] or 
					if logged_channels[k]['status'] != channels[k]['status']:
						self.writechannel(channels[k])
						logged_channels[k]['value'] = channels[k]['value']
						logged_channels[k]['status'] = channels[k]['status']
				except:
					self.writechannel(channels[k])
					logged_channels.setdefault(k, {})
					logged_channels[k].setdefault('value', channels[k]['value'])
					logged_channels[k].setdefault('status', channels[k]['status'])
				time.sleep(0.01)

################################################################################
#
# HTTP server handler
#
# Starts a web server on the configured port. Pointing a web browser to
#
#	 http://localhost:<configured-ip-address>
#
# opens a dynamic presentation of the channel values
# (automatic update once every second).
#
################################################################################

class HTTPServerHandler(BaseHTTPRequestHandler):

	# Overwrites the HTTP GET method of the base class.
	# Handles "every" request currently possible / allowed.
	def do_GET(self):
		try:
			if self.path.endswith('.png'): # The background image is to be loaded from the file system
				f = open(curdir + sep + self.path)
				self.send_response(200)
				self.send_header('Content-type', 'image/png')
				self.end_headers()
				self.wfile.write(f.read())
				f.close()
				return
			elif self.path.endswith('.div'): # The actual dynamic content is loaded into a <div> tag ;)
				self.send_response(200)
				self.send_header('Content-type', 'text/html')
				self.end_headers()
				self.wfile.write("<table style=\"border:none; border-spacing:5px; font-size: 20px;\">\n"
				+ "<thead>\n"
				+ "<tr>\n"
				+ "<th style=\"padding:5px;\">Room</th>\n"
				+ "<th style=\"padding:5px;\">Channel</th>\n"
				+ "<th style=\"padding:5px;\">Value</th>\n"
				+ "<th style=\"padding:5px;\">Status</th>\n"
				+ "</tr>\n"
				+ "</thead>\n"
				+ "<tbody>\n")
				# The channels are displayed sorted alphabetically by their room first, then by their channel name
				ch = channels.values()
				ch = sorted(ch, key = itemgetter('room', 'channel'))
				for channel in ch:
					self.wfile.write("<tr style=\"background-color: #%s;\">\n" % channel['color'])
					self.wfile.write("<td style=\"padding:5px;\">%s</td>\n" % channel['room'])
					self.wfile.write("<td style=\"padding:5px;\">%s</td>\n" % channel['channel'])
					self.wfile.write("<td style=\"padding:5px;\">%s</td>\n" % channel['value'])
					self.wfile.write("<td style=\"padding:5px;\">%s</td>\n</tr>\n" % channel['status'])
				self.wfile.write("</tbody>\n"
				+ "</table>\n")
				return				
			else: # The "static", basic HTML layout
				self.send_response(200)
				self.send_header('Content-type', 'text/html')
				self.end_headers()
				self.wfile.write("<html>\n"
				+ "<head>\n"
				+ "<title>Robot House Connector</title>\n"
				+ "<noscript>\n"
				+ "<meta http-equiv=\"refresh\" content=\"1\">\n"
				+ "</noscript>\n"
				+ "<script language=\"JavaScript\">\n"
				+ "function load(url, target) {\n"
				+ "	if (window.XMLHttpRequest) {\n"
				+ "		req = new XMLHttpRequest();\n"
				+ "	} else if (window.ActiveXObject) {\n"
				+ "		req = new ActiveXObject(\"Microsoft.XMLHTTP\");\n"
				+ "	}\n"
				+ "	if (req != undefined) {\n"
				+ "		req.onreadystatechange = function() {\n"
				+ "			loadDone(url, target);\n"
				+ "		};\n"
				+ "		req.open(\"GET\", url, true);\n"
				+ "		req.send(\"\");\n"
				+ "	}\n"
				+ "}\n"
				+ "function loadDone(url, target) {\n"
				+ "	if (req.readyState == 4) {\n"
				+ "		if (req.status == 200) {\n"
				+ "			document.getElementById(target).innerHTML = req.responseText;\n"
				+ "		} else {\n"
				+ "			document.getElementById(target).innerHTML=\"load error: \"+ req.status + \" \" +req.statusText;\n"
				+ "		}\n"
				+ "	}\n"
				+ "}\n"
				+ "var sURL = unescape(window.location.pathname);\n"
				+ "function doLoad() {\n"
				+ "	setTimeout(\"refresh()\", 1000);\n"
				+ "	refresh();\n"
				+ "}\n"
				+ "function refresh() {\n"
				+ "	load(\"index.div\", \"mydiv\");\n"
				+ "	setTimeout(\"refresh()\", 1000);\n"
				+ "	return false;\n"
				+ "}\n"
				+ "</script>\n"
				+ "</head>\n"
				+ "<body onload=\"doLoad()\" style=\"background:url(background.png) fixed center; font-family: Arial, Helvetica, sans-serif;\">\n"
				+ "<center>\n"
				+ "<br />\n"
				+ "<h1>Robot House Connector</h1>\n"
				+ "<br />\n"
				+ "<br />\n"
				+ "<div name=\"mydiv\" id=\"mydiv\"></div>\n"
				+ "<br />\n"
				+ "<p>&copy; 2010 by Patrick Neuberger</p>\n"
				+ "</center>\n"
				+ "</body>\n"
				+ "</html>\n")
				return
		except IOError:
			self.send_error(404, 'File not found: %s' % self.path)

	# Keeps the web server from flooding the console with debug messages
	def log_message(self, format, *args):
		return

################################################################################
#
# main(), Start-Up code
#
################################################################################

def main():
	try:
		print 'Shut down application with Ctrl-C'
		d = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
		print d
 
		_ZigBeeThread = ZigBeeThread()
		_ZigBeeThread.start()
		print 'Started ZigBee thread'
#		_MySQLThread = MySQLThread()
#		_MySQLThread.start()
#		print 'Started MySQL thread'
		_LoggerThread = LoggerThread()
		_LoggerThread.start()
		print 'Started Logger thread'
		server = HTTPServer(('', server_config['http_port']), HTTPServerHandler)
		print 'Started HTTP server'
		server.serve_forever()
		print 'Finished initialise'
	except KeyboardInterrupt:
		print '^C received'
		print 'Shutting down Logger thread'
		_LoggerThread.status = 0
		while _LoggerThread.is_alive():
			time.sleep(0.1)
 #	 print 'Shutting down MySQL thread'
 #	 _MySQLThread.status = 0
 #	 while _MySQLThread.is_alive():
 #		 time.sleep(0.1)
		print 'Shutting down ZigBee thread'
		_ZigBeeThread.status = 0
		while _ZigBeeThread.is_alive():
			time.sleep(0.1)
 
		print 'Shutting down HTTP server'
		server.socket.close()


if __name__ == '__main__':
	main()

### EOF ###