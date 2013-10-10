import roslib
import socket
import dynamic_reconfigure.client
import rospy

UDP_IP = "192.168.1.142"
UDP_PORT = 8042

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

rospy.init_node('squeezeme_speed_cfg', anonymous=True)
client = dynamic_reconfigure.client.Client("move_base/DWAPlannerROS/")
while True:
	data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
	print "received message:", data
	#pick pressure value from data (middle)
	splitted = data.split(" ")
	i = int(splitted[1]) /100
	#convert to reasonable values for trans and rot speed
	vel=(float(2)**i)/400
	theta =(float(2)**i)/350
	if vel > 0.45:
		vel=0.45
	print 'pseudo speed paramater is: ', vel, 'turning speed:', theta	
	params = {'max_vel_x' : vel, 'max_vel_y' : vel,'min_vel_y':-vel,'min_vel_y':-vel,'max_trans_vel':vel,'max_rot_vel':theta}
	config = client.update_configuration(params)
