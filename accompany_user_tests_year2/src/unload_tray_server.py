#! /usr/bin/env python

import roslib; roslib.load_manifest('accompany_user_tests_year2')
import rospy
import actionlib
import tf

from accompany_user_tests_year2.msg import *
from simple_script_server import *
sss = simple_script_server()

from moveit_commander import MoveGroupCommander
from simple_moveit_interface_accompany import *


class UnloadTrayServer:
	def __init__(self):
		self.server = actionlib.SimpleActionServer('unload_tray', UnloadTrayAction, self.execute, False)
		self.server.start()

	def execute(self, goal):
		sss.set_light("yellow")
		
		grasp = [-1.023739218711853, -1.0562658309936523, 2.3108131885528564, 1.5178372859954834, -0.08975735306739807, 1.0026973485946655, 0.1390783041715622]
		
		handle_arm = sss.move("arm",["intermediateback","intermediatefront",grasp],False)
		rospy.sleep(10)
		sss.move("sdh","cylopen")
		handle_arm.wait()

#		current_pose = moveit_get_current_pose("arm")
#		goal_pose1 = self.calculate_goal_pose(current_pose, 0.15, 0.1, -0.1, 0.0, 0.0, 0.6)
#		goal_pose2 = self.calculate_goal_pose(current_pose, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0)
#		success = moveit_cart_goals("arm", "base_link", [goal_pose1, goal_pose2], False)

#		sss.move("sdh","cylclosed")
#		current_pose = moveit_get_current_pose("arm")
#		goal_pose1 = self.calculate_goal_pose(current_pose, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0)
#		goal_pose2 = self.calculate_goal_pose(current_pose, 0.0, -0.2, 0.0, 0.0, 0.0, 0.0)
#		success = moveit_cart_goals("arm", "base_link", [goal_pose1, goal_pose2], False)


		sss.move("sdh","cylclosed")
		handle_arm = sss.move("arm",["intermediatefront"])

		table_height = 0.45
		intermediatefront_height = 0.98
		dz = table_height - intermediatefront_height

		current_pose = moveit_get_current_pose("arm")
		goal_pose1 = self.calculate_goal_pose(current_pose, -0.2, -0.3, dz, 0.0, 0.0, -1.5)
		if not moveit_cart_goals("arm", "base_link", [goal_pose1], False):
			sss.set_light("red")
			handle_arm = sss.move("arm",["intermediateback","folded"],False)
			self.server.set_aborted()
			return

		sss.move("sdh","cylopen")
		handle_arm = sss.move("arm",["intermediateback","folded"],False)
		rospy.sleep(3)
		sss.move("sdh","home")
		handle_arm.wait()

		sss.set_light("green")
		self.server.set_succeeded()

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
