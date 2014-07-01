#! /usr/bin/env python

import roslib;
roslib.load_manifest('accompany_user_tests_year2')
import rospy
import actionlib
import tf
import time

import dynamic_reconfigure.client
from std_srvs.srv import Empty


from accompany_user_tests_year2.msg import *
from simple_script_server import *
sss = simple_script_server()

from moveit_commander import MoveGroupCommander
#from simple_moveit_interface_accompany import *
rospy.init_node('my_node_name')
#sss.move("sdh","home")

local_costmap_dynamic_reconfigure_client = dynamic_reconfigure.client.Client("/local_costmap_node/costmap")

p21_troyes= [-0.0701594352722168, -1.4082157611846924, 1.6199039220809937, 1.0523432493209839, -0.4672948718070984, 0.3977415859699249, 0.9647517204284668]
p22_troyes= [2.2882933616638184, -1.2500675916671753, 2.0379726886749268, 1.8452539443969727, -0.4981473386287689, 1.5209976434707642, 0.8092219233512878]

p23_troyes= [2.3613481521606445, -1.5299841165542603, 2.2244346141815186, 1.7882975339889526, -0.14568567276000977, 1.2293986082077026, 0.4816061556339264]

#handle_arm = sss.move("arm",[p21_troyes],True)
#handle_arm = sss.move("arm",[[2.2882933616638184, -1.2500675916671753, 2.0379726886749268, 1.8452539443969727, -0.4981473386287689, 1.5209976434707642, 0.8092219233512878],"folded"],False)

#handle_arm = sss.move("arm",[p23_troyes,"folded"],False)
#handle_arm = sss.move("arm",[p22_troyes],True)
#handle_arm = sss.move("arm",["folded"],True)

#arm to higher position
#final
p24_troyes = [2.9579718112945557, -1.0143165588378906, 1.626341700553894, 1.0011065006256104, -3.06079363822937, -0.39361539483070374, 1.375096082687378]
#handle_arm = sss.move("arm",[p24_troyes],True)

#intermediate
p25_troyes = [2.7660703659057617, -1.7784847021102905, 2.5652642250061035, 0.7665566802024841, 0.16442710161209106, 0.9837681651115417, 0.19576311111450195]
#handle_arm = sss.move("arm",[p25_troyes],True)

#second choice
#intermediate 1
p26_troyes =[2.887775421142578, -0.6779797077178955, 2.4977622032165527, 1.5053181648254395, 0.15935616195201874, 1.371470332145691, 0.23653051257133484]

#intermediate 2
p27_troyes = [2.8876755237579346, -0.6779902577400208, 2.4924793243408203, 0.2725191116333008, 0.3797684907913208, 1.3752720355987549, 0.23653051257133484]
#final1

#final 2
p28_troyes = [2.912111282348633, -0.8155454993247986, 1.6875264644622803, 0.6187270879745483, 0.10598322749137878, 0.9390687346458435, 1.34400475025177]

p29_troyes =[2.946211814880371, -0.9938393235206604, 1.6254980564117432, 1.05857515335083, -3.062791109085083, -0.2328193485736847, 1.3772428035736084]
rospy.sleep(10)

p30_troyes =[2.946011781692505, -0.9943867921829224, 1.625508189201355, 0.17798008024692535, -3.062870979309082, -1.1878408193588257, 1.595353126525879]
p31_troyes =[2.946017026901245, -0.9943814873695374, 1.625508189201355, 1.0266671180725098, -3.062870979309082, -0.3526380956172943, 1.595353126525879]


#- Translation: [2.293, -2.979, 0.000]
#- Rotation: in Quaternion [0.000, 0.000, 0.757, 0.654]
#            in RPY [0.000, -0.000, 1.717]

#- Translation: [2.301, -2.934, 0.000]
#- Rotation: in Quaternion [0.000, 0.000, 0.756, 0.655]
#            in RPY [0.000, -0.000, 1.715]


            
#            - Translation: [2.290, -2.746, 0.000]
#- Rotation: in Quaternion [0.000, 0.000, 0.757, 0.653]
#            in RPY [0.000, -0.000, 1.718]
#- Translation: [2.260, -2.388, 0.000]
#- Rotation: in Quaternion [0.000, 0.000, 0.759, 0.651]
#            in RPY [0.000, -0.000, 1.724]
#- Translation: [2.235, -2.170, 0.000]
#- Rotation: in Quaternion [0.000, 0.000, 0.759, 0.652]
#            in RPY [0.000, -0.000, 1.722]


# adjust base footprint
local_config = local_costmap_dynamic_reconfigure_client.get_configuration(5.0)
local_costmap_dynamic_reconfigure_client.update_configuration({"footprint": "[[0.45,0.37],[0.45,-0.37],[-0.35,-0.37],[-0.35,0.37]]"})
rospy.sleep(0.5)
rospy.wait_for_service('/update_footprint')
try:
	req = rospy.ServiceProxy('/update_footprint', Empty)
	resp = req()
except rospy.ServiceException, e:
	print "Service call to /update_footprint failed: %s" % e

sss.move("base", [2.293, -2.388, 1.717],True,mode='linear')
sss.move("base", [2.293, -2.746, 1.717],True,mode='linear')
#sss.move("base", [2.293, -2.934, 1.717],True,mode='linear')
sss.move("base", [2.293, -2.879, 1.717],True,mode='linear')

# reset footprint
if local_config["footprint"] != None:
	local_costmap_dynamic_reconfigure_client.update_configuration({"footprint": local_config["footprint"]})
else:
	rospy.logwarn("Could not read previous local footprint configuration of /local_costmap_node/costmap, resetting to standard value: [[0.45,0.37],[0.45,-0.37],[-0.45,-0.37],[-0.45,0.37]].")
	local_costmap_dynamic_reconfigure_client.update_configuration({"footprint": "[[0.45,0.37],[0.45,-0.37],[-0.45,-0.37],[-0.45,0.37]]"})
rospy.sleep(0.5)
rospy.wait_for_service('/update_footprint')
try:
	req = rospy.ServiceProxy('/update_footprint', Empty)
	resp = req()
except rospy.ServiceException, e:
	print "Service call to /update_footprint failed: %s" % e


handle_arm = sss.move("arm",[p26_troyes],True)
handle_arm = sss.move("arm",[p27_troyes],True)


#handle_arm = sss.move("arm",[p29_troyes],True)
handle_arm = sss.move("arm",[p30_troyes],True)
sss.move("sdh","cylopen",True)
handle_arm = sss.move("arm",[p31_troyes],True)

#handle_arm = sss.move("arm",[p24_troyes],True)
##handle_arm = sss.move("arm",[p28_troyes],True)

sss.move("sdh","cylclosed",True)

sss.move("base", [2.293, -2.934, 1.721],True,mode='linear')
sss.move("base", [2.293, -2.746, 1.721],True,mode='linear')
sss.move("base", [2.293, -2.388, 1.721],True,mode='linear')
#sss.move("base", [2.293, -2.417, 0.000],True,mode='linear')



grasp  = [-1.023739218711853, -1.0562658309936523, 2.3108131885528564, 1.5178372859954834, -0.08975735306739807, 1.0026973485946655, 0.1390783041715622]
grasp2 = [-1.0235339403152466, -1.0605928897857666, 2.1852147579193115, 1.5356091260910034, -0.08200012892484665, 1.0033462047576904, 0.1387641429901123]
grasp3 = [-1.0245234966278076, -1.0662885904312134, 2.245345115661621, 1.543674111366272, -0.07828371226787567, 1.0038584470748901, 0.138010174036026]


p24_troyes = [2.9579718112945557, -1.0143165588378906, 1.626341700553894, 1.0011065006256104, -3.06079363822937, -0.39361539483070374, 1.375096082687378]

##good
rospy.sleep(10)
sss.move("tray", "deliverup")

handle_arm = sss.move("arm",[p24_troyes],True)
rospy.sleep(1)
handle_arm = sss.move("arm",["intermediateback"],True)
rospy.sleep(1)
handle_arm = sss.move("arm",["intermediatefront", grasp3],True)
rospy.sleep(1)
sss.move("sdh","cylopen")
rospy.sleep(1)

handle_arm = sss.move("arm",[grasp3,"intermediatefront"],True)
sss.move("sdh","home")
handle_arm = sss.move("arm",["intermediatefront","intermediateback"],True)
rospy.sleep(1)
handle_arm = sss.move("arm",["folded"],True)
##


#sss.move("base", [2.502, -2.031, 0])

#sss.move("base", [2.517, -1.975, 1.585])
#rospy.sleep(5)
#sss.move("base", [2.539, -2.831, 1.585])

#base position [2.557, -2.855, 0.000]
# Rotation: in Quaternion [0.000, 0.000, 0.731, 0.682]
#           in RPY [0.000, -0.000, 1.639]

#- Translation: [2.502, -2.031, 0.000]
#- Rotation: in Quaternion [0.000, 0.000, 0.736, 0.677]
#            in RPY [0.000, -0.000, 1.654]

#handle_arm = sss.move("arm",["folded", p21_troyes],True)
#handle_arm = sss.move("arm",[[2.2882933616638184, -1.2500675916671753, 2.0379726886749268, 1.8452539443969727, -0.4981473386287689, 1.5209976434707642, 0.8092219233512878]],True)
#handle_arm = sss.move("arm",[p21_troyes],True)
