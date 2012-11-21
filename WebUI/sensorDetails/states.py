
class States(object):
    exposed = True
    
    def GET(self, *args, **kwargs):
        
    # Overwrites the HTTP GET method of the base class.
    # Handles "every" request currently possible / allowed.
    def do_GET(self):
        try:
            if self.path.endswith('.div'): # The actual dynamic content is loaded into a <div> tag ;)
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

    def handler_contact_reed_color(self, channel_uuid, value):
        if value == '1':
            return 'FF0000'
        else:
            return '00FF00' # closed door == green colour

    def handler_contact_pressuremat_color(self, channel_uuid, value):
        if value == '1':
            return '00FF00' # vacant chair == green colour
        else:
            return 'FF0000'

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
    
