from moveit_commander import MoveGroupCommander
from simple_moveit_interface_accompany import *

if __name__ == "__main__":

    rospy.init_node("moveit_accompany")
    #goal = get_goal_from_server("arm", "pregrasp")
    #moveit_joint_goal("arm", goal)
    current_pose = moveit_get_current_pose("arm")
    print current_pose
    
    goal_pose = current_pose.pose
    goal_pose.position.z += 0.2
    
    print moveit_cart_goals("arm", "base_link", [goal_pose], False)
    
