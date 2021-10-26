#!/usr/bin/env python

# import relevant libraries
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# import python modules
import numpy as np
import math

# constants
FREQUENCY = 10 # Hz
LIN_VEL = 0.2 # m/s
ANG_VEL = 0.2 # rad/s

DEFAULT_CMD_VEL = 'cmd_vel'
DEFAULT_SCAN = 'scan'
DEFAULT_ODOM = 'odom'

class ROSBOT():
    def __init__(self):

        """ Constructor """
        self.rate = rospy.Rate(FREQUENCY)

        # publisher and subscriber
        self._cmd_pub = rospy.Publisher(DEFAULT_CMD_VEL, Twist, queue_size=1)
        self._laser_sub =rospy.Subscriber(DEFAULT_SCAN, LaserScan, self._laser_callback, queue_size=1)
        

    def move(self, lin_vel, ang_vel):
        """Send a velocity command (linear vel in m/s, angular vel in rad/s)."""
        twist_msg = Twist()
        twist_msg.linear.x = lin_vel
        twist_msg.angular.z = ang_vel

        self._cmd_pub.publish(twist_msg)

    def stop(self):
        """ stop the robot """
        twist_msg = Twist()
        self._cmd_pub.publish(twist_msg)


    def spin(self):
        while not rospy.is_shutdown():
            self.move(LIN_VEL, 0)

            self.rate.sleep()