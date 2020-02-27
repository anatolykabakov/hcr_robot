#!/usr/bin/env python

"""
ROS node for HCR Robot.
"""

import roslib; roslib.load_manifest("robot_node")
import rospy
from math import sin,cos
import numpy as np

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
import tf

from arduino import protocol

class robot_node:

    def __init__(self):
        """ Start up connection to the HCR Robot. """
        rospy.init_node('robot_node')

        arduino_port = rospy.get_param('~arduino_port')
        arduino_rate = rospy.get_param('~arduino_rate', '')
        tf_prefix = rospy.get_param('~tf_prefix')

        rospy.loginfo("Arduino Using port: %s"%(arduino_port))
        rospy.loginfo("Arduino Using rate: %s"%(arduino_rate))
        rospy.loginfo("tf_prefix: %s"%(tf_prefix))

        self.robot = protocol(arduino_port, arduino_rate)

        rospy.Subscriber("cmd_vel", Twist, self.cmdVelCb)
        
        self.odomPub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.odomBroadcaster = TransformBroadcaster()

        self.cmd_vel = [0,0] 

        self.frame_id_tf = ""
        self.child_frame_id_tf = ""

        if tf_prefix != "":
            self.frame_id_tf = tf_prefix + "/" + 'odom'
            self.child_frame_id_tf = tf_prefix + "/" +'base_link'
        else:
            self.frame_id_tf = 'odom'
            self.child_frame_id_tf = 'base_link'

    def spin(self):                
        # main loop of driver
        r = rospy.Rate(100)

        while not rospy.is_shutdown():
            # get motor velocity values
            x, y, yaw, v, w, vr, vl = self.robot.getOdometry()

            orientation_q = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
        
            odom = Odometry()
            odom.header.frame_id = self.frame_id_tf
            odom.child_frame_id = self.child_frame_id_tf
            odom.header.stamp = rospy.Time.now()
            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation.w = orientation_q[3]
            odom.pose.pose.orientation.x = orientation_q[0]
            odom.pose.pose.orientation.y = orientation_q[1]
            odom.pose.pose.orientation.z = orientation_q[2]
            odom.twist.twist.linear.x = v
            odom.twist.twist.angular.z = w
            
            # publish odom
            self.odomPub.publish(odom)

            # publish tf
            self.odomBroadcaster.sendTransform((x, y, 0), 
                                                orientation_q,
                                                rospy.Time.now(), 
                                                self.child_frame_id_tf, 
                                                self.frame_id_tf)
            
            # send goal velocity 
            self.robot.setMotors(self.cmd_vel[0], self.cmd_vel[1])
            r.sleep()
            

        # shut down
        self.robot.stop()

    def cmdVelCb(self,req):
        # send updated movement commands
        self.cmd_vel = [req.linear.x,req.angular.z] 

if __name__ == "__main__":    
    robot = robot_node()
    robot.spin()

