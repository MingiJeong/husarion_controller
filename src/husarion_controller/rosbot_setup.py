#!/usr/bin/env python

# import relevant libraries
import rospy

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

# import python modules
import numpy as np
import math

# constants
FREQUENCY = 10 # Hz
LIN_VEL = 0.2 # m/s
ANG_VEL = math.pi/18 # rad/s

DEFAULT_CMD_VEL = 'cmd_vel'
DEFAULT_SCAN = 'scan'
DEFAULT_ODOM = 'odom'

class ROSBOT():
    def __init__(self):

        """ Constructor """
        self.rate = rospy.Rate(FREQUENCY)
        self.heading = None

        # publisher and subscriber
        self._cmd_pub = rospy.Publisher(DEFAULT_CMD_VEL, Twist, queue_size=1)

        self._odom_sub = rospy.Subscriber(DEFAULT_ODOM, Odometry, self._odom_callback, queue_size=1)
        self._laser_sub = rospy.Subscriber(DEFAULT_SCAN, LaserScan, self._laser_callback, queue_size=1)
        self.lin_vel = LIN_VEL
        self.ang_vel = ANG_VEL


    def _laser_callback(self, laser_msg):
        pass

    
    def _odom_callback(self, odom_msg):
        # heading update
        self.heading = self.get_heading(odom_msg)


    def get_heading(self, odom_msg):
        orientation_q = odom_msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll,pitch,yaw) = euler_from_quaternion(orientation_list)

        return yaw


    def move(self, lin_vel, ang_vel):
        """Send a velocity command (linear vel in m/s, angular vel in rad/s)."""
        twist_msg = Twist()
        twist_msg.linear.x = self.lin_vel
        twist_msg.angular.z = self.ang_vel

        self._cmd_pub.publish(twist_msg)

    def stop(self):
        """ stop the robot """
        twist_msg = Twist()
        self._cmd_pub.publish(twist_msg)


    def translate(self, distance):
        """
        Function to move forward for a given distance
        """
        twist_msg = Twist()
        twist_msg.linear.x = self.lin_vel

        start_time = rospy.get_rostime()
        duration = rospy.Duration(distance / self.lin_vel)

        while not rospy.is_shutdown():
            if rospy.get_rostime() - start_time >= duration:
                break

            else:
                self._cmd_pub.publish(twist_msg)
                self.rate.sleep()

        # stopper
        self.stop()


    def rotate_rel(self, angle):
        """
        Rotate in place the robot of rotation_angle (rad) based on fixed velocity.
        depending on the rotation direction (sign of angle)
        """

        twist_msg = Twist()
        # twist_msg.angular.z = self.ang_vel

        if angle >= 0: # counter-clockwise True
            twist_msg.angular.z = self.ang_vel
        else: # counter-clockwise False, i.e., clockwise
            twist_msg.angular.z = - self.ang_vel

        start_time = rospy.get_rostime()
        duration = rospy.Duration(abs(angle) / self.ang_vel)

        while not rospy.is_shutdown():
            if rospy.get_rostime() - start_time >= duration:
                break

            else:
                self._cmd_pub.publish(twist_msg)
                self.rate.sleep()
        
        # stopper
        self.stop()


    def rotate_abs(self, dest_angle):
        """
        Rotate in place the robot of rotation_angle (rad) based on fixed velocity.
        depending on the rotation direction (sign of angle)
        """

        twist_msg = Twist()
        angle_to_go = dest_angle - self.heading

        if angle_to_go >= 0: # counter-clockwise True
            twist_msg.angular.z = self.ang_vel
        else: # counter-clockwise False, i.e., clockwise
            twist_msg.angular.z = - self.ang_vel

        start_time = rospy.get_rostime()
        duration = rospy.Duration(abs(angle_to_go) / self.ang_vel)

        while not rospy.is_shutdown():
            if rospy.get_rostime() - start_time >= duration:
                break

            else:
                self._cmd_pub.publish(twist_msg)
                self.rate.sleep()
        
        # stopper
        self.stop()


    def spin(self):
        # while not rospy.is_shutdown():
        try: 
            self.translate(1)
            rospy.loginfo("translation finish")

            self.rotate_rel(math.pi/6)
            rospy.loginfo("rotation 1 finish")

            self.rotate_rel(- math.pi/6)
            rospy.loginfo("rotation 2 finish")


            self.rotate_abs(math.pi * 3/4)
            rospy.loginfo("rotation abs finish")

            # self.rate.sleep()


        except rospy.ROSInterruptException:
            rospy.logerr("ROS node interrupted.")
