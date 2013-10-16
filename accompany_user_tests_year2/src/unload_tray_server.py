#! /usr/bin/env python

import roslib; roslib.load_manifest('accompany_user_tests_year2')
import rospy
import actionlib
import tf
import time

from accompany_user_tests_year2.msg import *
from simple_script_server import *
sss = simple_script_server()

#from moveit_commander import MoveGroupCommander
#from simple_moveit_interface_accompany import *


class UnloadTrayServer:
	def __init__(self):
		self.server = actionlib.SimpleActionServer('unload_tray', UnloadTrayAction, self.execute, False)
		self.server.start()

	def execute(self, goal):
		if goal.table_height <= 0.445 or goal.table_height >= 0.605:
			rospy.logerr("table_height (" + str(goal.table_height) + ") not within limits (0.45...0.60), goal aborted.")
			self.server.set_aborted()
			return
		#turn to user
		if goal.table_height == 0.45:
	 		sss.move("torso", [[-0.08,0.17,-0.08]])
	 
			print "placing object on table with a height of " + str(goal.table_height)
		
			grasp = [-1.023739218711853, -1.0562658309936523, 2.3108131885528564, 1.5178372859954834, -0.08975735306739807, 1.0026973485946655, 0.1390783041715622]
		
			handle_arm = sss.move("arm",["intermediateback","intermediatefront",grasp],False)
			#handle_arm = sss.move("arm",["intermediatefront"],False)
			rospy.sleep(10)
			sss.move("sdh","cylopen")
			handle_arm.wait()

			sss.move("sdh","cylclosed")
			handle_arm = sss.move("arm",["intermediatefront"])

			#table_height = goal.table_height
			#intermediatefront_height = 0.98
			#dz = table_height - intermediatefront_height

		

		
			#if not moveit_cart_goals("arm", "base_link", [goal_pose1], False) == "succeeded":
			#	sss.set_light("red")
			#	sss.move("tray","deliverdown")
	#			handle_arm = sss.move("arm",["intermediateback","folded"])
			#	self.server.set_aborted()
			#	return

			p1 = [-0.0677609748672694, -0.45803928788518533, 1.754957867320627, 1.9358435459434986, -0.5034669896413106, 0.7991892780410126, 0.3653704138705507]
			p2 = [0.09568882174789906, -0.5366700394079089, 1.721393596555572, 1.9865455229301006, -0.5129501838291617, 0.7132017726544291, 0.49976619239896536]
			p3 = [0.22279815725050867, -0.6564056833740324, 1.7153943999437615, 2.0081158993853023, -0.5094200487947091, 0.6249831056920812, 0.6295777186751366]
		
			p4 = [0.31678347277920693, -0.8026507119648159, 1.726887084543705, 1.9964563865214586, -0.48470462061231956, 0.5470565221039578, 0.7486096813809127]
			p5 =[0.37407077802345157, -0.9328190083615482, 1.7374236889445456, 1.9560632872162387, -0.4657182568917051, 0.49778644542675465, 0.8369000980164856]
			p6 =[0.42153078370029107, -1.0870567939709872, 1.742178603359207, 1.8818300997372717, -0.4450125983566977, 0.46022555319359526, 0.932977354619652]
			p7 =[0.4413558242376894, -1.1869079691823572, 1.7376993048819713, 1.8183607151731849, -0.43327091314131394, 0.4457910807977896, 0.989452947396785]
			p8 =[0.4546498202398652, -1.3045525378547609, 1.7218842049478553, 1.7260430962778628, -0.4221305515966378, 0.44235596476690375, 1.0536736289504915]
			p9 =[0.4551111281325575, -1.4165355968289077, 1.6960962603334337, 1.6181875145994127, -0.41435565701976884, 0.4444336187443696, 1.113348227692768]
			p10 =[0.4396249196724966, -1.5163538900669664, 1.6609547425759956, 1.4999713725410402, -0.4107645494332246, 0.46756143373204395, 1.1681542885489762]
			p11 =[0.42196490650530905, -1.575385732576251, 1.633163999649696, 1.415779996663332, -0.41062599208089523, 0.48633785598212853, 1.1998219589004293]
			p12 =[0.4018593500368297, -1.6224102401174605, 1.6058433627476916, 1.3371890399139374, -0.4107816547038965, 0.5068991582957096, 1.2285655857995152]
			p13 =[0.3740939331401023, -1.6692602596667712, 1.5726137426609057, 1.2441861303086625, -0.41711411235337437, 0.5347730094872531, 1.2570846655944479]
			p14 =[0.37384849786758423, -1.6693923473358154, 1.5724937915802002, 1.2437162399291992, -0.417205810546875, 0.5350838303565979, 1.258846640586853]


			p15=[-0.5151359438896179, -1.5682907104492188, 1.935489296913147, 0.42959874868392944, -0.7945099472999573, 0.5764082670211792, 1.2594016790390015]
			p16= [-0.5146042704582214, -1.6116349697113037, 1.917541742324829, 0.353283554315567, -0.7945554852485657, 0.6055818796157837, 1.259820580482483]
			p17= [-0.3796073794364929, -1.3648346662521362, 1.9743826389312744, 0.7521292567253113, -0.7862803339958191, 0.5338812828063965, 1.2719261646270752]
			p18= [-0.28245383501052856, -0.818909227848053, 1.8353912830352783, 1.3951565027236938, -0.49456751346588135, 0.18976472318172455, 0.6043481826782227]

			#handle_arm = sss.move("arm",["intermediatefront", p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15])
			handle_arm = sss.move("arm",["intermediatefront", p18, p16])
			#rospy.sleep(3)
			#sss.trigger("arm", "stop")
	###
		
			#current_pose = moveit_get_current_pose("arm")
			#goal_pose1 = self.calculate_goal_pose(current_pose, 0.05, 0.05, 0.0, 0.0, 0.0, 0.0)
		
	######
			sss.move("sdh","cylopen")
			handle_arm = sss.move("arm",[p17],True)
			#rospy.sleep(1)
			handle_arm = sss.move("arm",["intermediateback","folded"],False)
			rospy.sleep(4)
			sss.move("sdh","home")
			sss.move("torso","home")
	######
			#sss.move("tray","store")
			#handle_arm.wait()

			#sss.set_light("green")
			#self.server.set_succeeded()
		else:
			sss.move("arm",["intermediateback","intermediatefront", [-1.5351587533950806, -0.4335061311721802, 2.7396888732910156, 1.5267207622528076, -0.6499853134155273, 0.2883549630641937, 0.025645868852734566]],False)

	def calculate_goal_pose(self, current_pose, dx, dy, dz, droll, dpitch, dyaw):
		goal_pose = current_pose.pose
		goal_pose.position.x += dx
		goal_pose.position.y += dy
		goal_pose.position.z += dz
		rpy = tf.transformations.euler_from_quaternion([goal_pose.orientation.x, goal_pose.orientation.y, goal_pose.orientation.z, goal_pose.orientation.w])
		rpy = [rpy[0],rpy[1],rpy[2]]
		rpy[0] += droll
		rpy[1] += dpitch
		rpy[2] += dyaw
		quat = tf.transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2])
		goal_pose.orientation.x = quat[0]
		goal_pose.orientation.y = quat[1]
		goal_pose.orientation.z = quat[2]
		goal_pose.orientation.w = quat[3]
		return goal_pose
		

if __name__ == '__main__':
	rospy.init_node('unload_tray_server')
	server = UnloadTrayServer()
	rospy.spin()
