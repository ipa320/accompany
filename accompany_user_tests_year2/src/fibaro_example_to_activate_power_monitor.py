#!/usr/bin/python
from gi.repository import Gtk
import urllib2, base64

class MyWindow(Gtk.Window):

    def __init__(self):
        Gtk.Window.__init__(self, title="Fibaro home center relay controller")
	self.set_border_width(10)
	
	hbox = Gtk.Box(spacing=6)
	self.add(hbox)
	
	button = Gtk.Button("On")
        button.connect("clicked", self.on_button_clicked)
        hbox.pack_start(button, True, True, 0)

        button = Gtk.Button("Off")
        button.connect("clicked", self.off_button_clicked)
        hbox.pack_start(button, True, True, 0)

	self._onUrl = "http://%(ip)s:80/api/callAction?deviceID=4&name=turnOn" % {'ip': "192.168.1.109"}
	self._offUrl = "http://%(ip)s:80/api/callAction?deviceID=4&name=turnOff" % {'ip': "192.168.1.109"}
    def on_button_clicked(self, widget):
        print "Turned on"
	url = self._onUrl
	request = urllib2.Request(url)
	base64string = base64.encodestring('%s:%s' % ('admin', 'admin')).replace('\n', '')
	request.add_header("Authorization", "Basic %s" % base64string)
	result = urllib2.urlopen(request)

    def off_button_clicked(self, widget):
        print "Turned off"
	url = self._offUrl
	request = urllib2.Request(url)
	base64string = base64.encodestring('%s:%s' % ('admin', 'admin')).replace('\n', '')
	request.add_header("Authorization", "Basic %s" % base64string)
	result = urllib2.urlopen(request)

win = MyWindow()
win.connect("delete-event", Gtk.main_quit)
win.show_all()
Gtk.main()

