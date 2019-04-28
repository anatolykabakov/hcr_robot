#!/usr/bin/env python

"""
ROS node for HCR Robot.
"""

import roslib; roslib.load_manifest("hcr_node")
import rospy
from math import sin,cos
import numpy as np

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster

from hcr_driver.hcr_driver import hcr, WHEELS_DIST, WHEELS_RAD, MAX_SPEED
#Arduino serial port
ARDUINO_PORT = '/dev/ttyACM0'
ARDUINO_SPEED = 115200

#RPLidar serial port
LIDAR_PORT = '/dev/ttyUSB0'

class HCRNode:

    def __init__(self):
        """ Start up connection to the HCR Robot. """
        rospy.init_node('hcr')

        self.port = rospy.get_param('~port', ARDUINO_PORT)
        rospy.loginfo("Using port: %s"%(self.port))

        self.robot = hcr(self.port, LIDAR_PORT)

        rospy.Subscriber("cmd_vel", Twist, self.cmdVelCb)
        self.scanPub = rospy.Publisher('base_scan', LaserScan, queue_size=10)
        self.odomPub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.odomBroadcaster = TransformBroadcaster()

        self.cmd_vel = [0,0] 

    def spin(self):        
        encoders = [0,0]

        self.x = 0                  # position in xy plane
        self.y = 0
        self.th = 0
        then = rospy.Time.now()

        # things that don't ever change
        scan_link = rospy.get_param('~frame_id','base_laser_link')
        scan = LaserScan(header=rospy.Header(frame_id=scan_link)) 
        scan.angle_min = 0
        scan.angle_max = 6.26
        scan.angle_increment = 0.017437326
        scan.range_min = 0.020
        scan.range_max = 5.0
        odom = Odometry(header=rospy.Header(frame_id="odom"), child_frame_id='base_link')
    
        # main loop of driver
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            # prepare laser scan
            scan.header.stamp = rospy.Time.now()    
            #self.robot.requestScan()
            scan.ranges = self.robot.getScanRanges()

            # get motor encoder values
            vr, vl = self.robot.getMotors()

            # send updated movement commands
            self.robot.setMotors(self.cmd_vel[0], self.cmd_vel[1])
            
            
            # now update position information
            dt = (scan.header.stamp - then).to_sec()
            then = scan.header.stamp

            omegaRight = vr/WHEELS_RAD
            omegaLeft  = vl/WHEELS_RAD
            # фактическая линейная скорость центра робота
            linear_velocity = (WHEELS_RAD/2)*(omegaRight + omegaLeft);#//m/s
            # фактическая угловая скорость поворота робота
            angular_velocity = (WHEELS_RAD/WHEELS_DIST)*(omegaRight - omegaLeft);
            self.th+=(angular_velocity * dt)#;  #  // направление в рад
            self.th = normalize_angle(self.th)
            self.x += linear_velocity*cos(self.th) * dt# // в метрах
            self.y += linear_velocity*sin(self.th) * dt#

            # prepare tf from base_link to odom
            quaternion = Quaternion()
            quaternion.z = sin(self.th/2.0)
            quaternion.w = cos(self.th/2.0)

            # prepare odometry
            odom.header.stamp = rospy.Time.now()
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = quaternion
            odom.twist.twist.linear.x = linear_velocity
            odom.twist.twist.angular.z = angular_velocity

            # publish everything
            self.odomBroadcaster.sendTransform( (self.x, self.y, 0), (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                then, "base_link", "odom" )
            self.scanPub.publish(scan)
            self.odomPub.publish(odom)

            # wait, then do it again
            r.sleep()

        # shut down
        self.robot.stop()

    def cmdVelCb(self,req):
        vLinear = req.linear.x 
        vAngular = req.angular.z * (WHEELS_DIST/2)
        vr = ((2 * vLinear) + (WHEELS_DIST * vAngular)) / 2
        vl = ((2 * vLinear) - (WHEELS_DIST * vAngular))/ 2
        k = max(abs(vr),abs(vl))
        # sending commands higher than max speed will fail
        if k > MAX_SPEED:
            vr = vr*MAX_SPEED/k; vl = vl*MAX_SPEED/k
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
    robot = HCRNode()
    robot.spin()

