import socket
from time import sleep

UDP_IP = "192.168.1.142"
UDP_PORT = 8042
MESSAGE = "100 100 100"

print "UDP target IP:", UDP_IP
print "UDP target port:", UDP_PORT
print "message:", MESSAGE


sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
while True:
    sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
    sleep(2)
