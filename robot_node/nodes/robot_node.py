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

from arduino import protocol

class robot_node:

    def __init__(self):
        """ Start up connection to the HCR Robot. """
        rospy.init_node('robot_node')

        arduino_port = rospy.get_param('~arduino_port')
        arduino_rate = rospy.get_param('~arduino_rate', '')

        self.robot = protocol(arduino_port, arduino_rate)

        self.wheel_dist = rospy.get_param('~wheel_dist')
        self.wheel_radius = rospy.get_param('~wheel_radius')
        self.max_speed = rospy.get_param('~max_speed')
        self.tf_prefix = rospy.get_param('~tf_prefix')

        rospy.loginfo("Arduino Using port: %s"%(arduino_port))
        rospy.loginfo("Arduino Using rate: %s"%(arduino_rate))
        rospy.loginfo("wheel_dist: %s"%(self.wheel_dist))
        rospy.loginfo("wheel_radius: %s"%(self.wheel_radius))
        rospy.loginfo("max_speed: %s"%(self.max_speed))
        rospy.loginfo("tf_prefix: %s"%(self.tf_prefix))

        rospy.Subscriber("cmd_vel", Twist, self.cmdVelCb)
        self.odomPub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.odomBroadcaster = TransformBroadcaster()

        self.cmd_vel = [0,0] 

    def spin(self):        
        encoders = [0,0]

        self.x = 0                  # position in xy plane
        self.y = 0
        self.th = 0
        then = rospy.Time.now()
        if self.tf_prefix != "":
            frame_id_tf = self.tf_prefix + "/" + "odom"
            child_frame_id_tf = self.tf_prefix + "/" +'base_link'
        else:
            frame_id_tf = "odom"
            child_frame_id_tf = 'base_link'
        odom = Odometry(header=rospy.Header(frame_id=frame_id_tf), child_frame_id=child_frame_id_tf)
    
        # main loop of driver
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            odom.header.stamp = rospy.Time.now()
            # get motor velocity values
            vr, vl = self.robot.getMotors()

            # send updated movement commands
            self.robot.setMotors(self.cmd_vel[0], self.cmd_vel[1])
            
            
            # now update position information
            dt = (odom.header.stamp - then).to_sec()
            then = odom.header.stamp
            
            #odometry navigation
            omegaRight = vr/self.wheel_radius
            omegaLeft  = vl/self.wheel_radius
            linear_velocity = (self.wheel_radius/2)*(omegaRight + omegaLeft)
            angular_velocity = (self.wheel_radius/self.wheel_dist)*(omegaRight - omegaLeft)
            self.th+=(angular_velocity * dt)
            self.th = normalize_angle(self.th)
            self.x += linear_velocity*cos(self.th) * dt
            self.y += linear_velocity*sin(self.th) * dt

            # prepare tf from base_link to odom
            quaternion = Quaternion()
            quaternion.z = sin(self.th/2.0)
            quaternion.w = cos(self.th/2.0)

            # prepare odometry
            #odom.header.stamp = rospy.Time.now()
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = quaternion
            odom.twist.twist.linear.x = linear_velocity
            odom.twist.twist.angular.z = angular_velocity

            # publish everything
            self.odomBroadcaster.sendTransform( (self.x, self.y, 0), (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                then, child_frame_id_tf, frame_id_tf )
            self.odomPub.publish(odom)

            # wait, then do it again
            r.sleep()

        # shut down
        self.robot.stop()

    def cmdVelCb(self,req):
        vLinear = req.linear.x 
        vAngular = req.angular.z
        vr = ((2 * vLinear) + (self.wheel_dist * vAngular))/2
        vl = ((2 * vLinear) - (self.wheel_dist * vAngular))/2
        k = max(abs(vr),abs(vl))
        # sending commands higher than max speed will fail
        if k > self.max_speed:
            vr = vr*self.max_speed/k; vl = vl*self.max_speed/k
        self.cmd_vel = [ round(vr,1), round(vl,1) ]

def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].

    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle


if __name__ == "__main__":    
    robot = robot_node()
    robot.spin()

