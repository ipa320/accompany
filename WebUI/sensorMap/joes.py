
################################################################################
#
# HTTP server handler
#
# Starts a web server on the configured port. Pointing a web browser to
#
#     http://localhost:<configured-ip-address>
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
                + "    if (window.XMLHttpRequest) {\n"
                + "        req = new XMLHttpRequest();\n"
                + "    } else if (window.ActiveXObject) {\n"
                + "        req = new ActiveXObject(\"Microsoft.XMLHTTP\");\n"
                + "    }\n"
                + "    if (req != undefined) {\n"
                + "        req.onreadystatechange = function() {\n"
                + "            loadDone(url, target);\n"
                + "        };\n"
                + "        req.open(\"GET\", url, true);\n"
                + "        req.send(\"\");\n"
                + "    }\n"
                + "}\n"
                + "function loadDone(url, target) {\n"
                + "    if (req.readyState == 4) {\n"
                + "        if (req.status == 200) {\n"
                + "            document.getElementById(target).innerHTML = req.responseText;\n"
                + "        } else {\n"
                + "            document.getElementById(target).innerHTML=\"load error: \"+ req.status + \" \" +req.statusText;\n"
                + "        }\n"
                + "    }\n"
                + "}\n"
                + "var sURL = unescape(window.location.pathname);\n"
                + "function doLoad() {\n"
                + "    setTimeout(\"refresh()\", 1000);\n"
                + "    refresh();\n"
                + "}\n"
                + "function refresh() {\n"
                + "    load(\"index.div\", \"mydiv\");\n"
                + "    setTimeout(\"refresh()\", 1000);\n"
                + "    return false;\n"
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
