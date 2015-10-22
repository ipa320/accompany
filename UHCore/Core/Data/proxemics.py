#!/usr/bin/python

#Add project reference
import sys, os
path = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../')
sys.path.append(path)

import math

class ProxemicMover(object):
    """Python wrapper/interface for User Proxemics Service """    
    def __init__(self, robot):
        self._robot = robot
        import Robots.rosHelper
        Robots.rosHelper.ROS.configureROS(packageName='accompany_proxemics')
        ros = Robots.rosHelper.ROS()
        ros.initROS()
        import rospy
        import tf
        import accompany_context_aware_planner.srv
        import geometry_msgs.msg
        self._geoMsg = geometry_msgs.msg.Pose
        self._srvMsg = accompany_context_aware_planner.srv.GetPotentialProxemicsLocations
        self._rospy = rospy
        self._tf = tf
    
    def gotoTarget(self, userId, posture, x, y, theta, taskId = 3):
        """
            Uses proxemics to attempt to locate a valid location to send the robot near the specified location.
            Raises exception on error
        """
        if self._robot == None:
            raise Exception("no robot specified")
        
        try:
            print "Waiting for accompany_proxemics"
            self._rospy.wait_for_service('accompany_context_aware_planner/get_potential_proxemics_locations', 5)
            print "Connected to accompany_proxemics"
        except self._rospy.ROSException:
            print >> sys.stderr, "get_potential_proxemics_locations not ready within timeout"
            raise Exception("proxemics module not running or broken")
        
        getProxemicLocation = self._rospy.ServiceProxy(
                                                       'accompany_context_aware_planner/get_potential_proxemics_locations', 
                                                       self._srvMsg)
        try:
            pose = self._geoMsg()
            quaternion = self._tf.transformations.quaternion_from_euler(0,0,math.radians(theta))
            pose.orientation.x = quaternion[0]
            pose.orientation.y = quaternion[1]
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]
            pose.position.x = x
            pose.position.y = y
            pose.position.z = 0
            
            response = getProxemicLocation(userId=userId,
                                            userPosture=posture, userPose=pose, robotGenericTaskId=taskId)
            
            if len(response.targetPoses) == 0:
                self._rospy.loginfo("No valid target pose was found.")
            else:
                self._rospy.loginfo("Response target Poses size is: %d" % (len(response.targetPoses)))
                self._rospy.loginfo("Request is:  x=%f, y=%f, z=%f yaw=%f" % (x, y, 0, theta))
                
                for target in response.targetPoses:
                    self._rospy.loginfo("MsgSeq=%d, time=%2f, coordinate frame=%s " % (
                                                                                    target.header.seq,
                                                                                    self._rospy.Time.now().to_sec() - target.header.stamp.to_sec(),
                                                                                    target.header.frame_id))
                    
                    (_, _, yaw) = self._tf.transformations.euler_from_quaternion(
                                                                                 [target.pose.orientation.x, 
                                                                                  target.pose.orientation.y, 
                                                                                  target.pose.orientation.z, 
                                                                                  target.pose.orientation.w])
                    self._rospy.loginfo("Response is: x=%f, y=%f, z=%f yaw=%f" % (
                                                                                target.pose.position.x,
                                                                                target.pose.position.y,
                                                                                target.pose.position.z,
                                                                                math.degrees(yaw)))
                    location = [target.pose.position.x, target.pose.position.y, yaw]
                    if self._robot.setComponentState('base', location) in [3, 'SUCCEEDED']:
                        return True

        except self._rospy.ServiceException, e:
            print "Service did not process request: %s" % str(e)
        
        return False

if __name__ == "__main__":
    from optparse import OptionParser
    parser = OptionParser('proxemics.py userId userPosture X(m) Y(m) Orientation(deg) taskId')
    (_, args) = parser.parse_args()
    if(len(args) != 6):
        parser.error("incorrect number of arguments")
        
    from Robots.robotFactory import Factory
    r = Factory.getCurrentRobot()
    if r != None:
        p = ProxemicMover(r)
        p.gotoTarget(int(args[0]), int(args[1]), float(args[2]), float(args[3]), float(args[4]), int(args[5]))    
    
