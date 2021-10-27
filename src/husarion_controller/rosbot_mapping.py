#!/usr/bin/env python

"""
    rosbot_mapping.py

    purpose: 
    - Occupancy grid mapping by rosbot
"""

# import relevant libraries
from husarion_controller.rosbot_setup import DEFAULT_CMD_VEL
import rospy
import tf
import tf2_ros
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import geometry_msgs.msg

# import python modules
import numpy as np
import math
import sys

# constants
MAP_FREQ = 10 # Hz

DEFAULT_MAP_TOPIC = "map"
DEFAULT_MAP_FRAME = "map"
DEFAULT_ODOM_FRAME = "odom"
DEFAULT_SCAN = 'scan'


class Mapper():
    def __init__(self):
        """ constructor """
        self.rate = rospy.Rate(MAP_FREQ)

        # publisher and subscriber
        self._map_pub = rospy.Publisher(DEFAULT_MAP_TOPIC, OccupancyGrid, queue_size=1)
        self._laser_sub = rospy.Subscriber(DEFAULT_SCAN, LaserScan, self._laser_callback, queue_size=1)

        self.resolution = rospy.get_param("map_resolution", 0.05) # m/cell
        self.width = rospy.get_param("width", 100) # cell
        self.height = rospy.get_param("hegiht", 100) # cell
        self.transform_data = (- self.resolution * self.width // 2, 
                               - self.resolution * self.height // 2,
                               0, 0, 0)


    def _laser_callback(self, laser_msg):
        # do something for mapping
        self.build_map()

    def static_broadcaster(self):
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        static_transformStamped = geometry_msgs.msg.TransformStamped()

        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = DEFAULT_ODOM_FRAME
        static_transformStamped.child_frame_id = DEFAULT_MAP_FRAME

        static_transformStamped.transform.translation.x = self.transform_data[0]
        static_transformStamped.transform.translation.y = self.transform_data[1]
        static_transformStamped.transform.translation.z = 0

        quat = tf.transformations.quaternion_from_euler(
                   self.transform_data[2],self.transform_data[3],self.transform_data[4])
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        broadcaster.sendTransform(static_transformStamped)


    def build_map(self):
        map_msg = OccupancyGrid()

        # header
        map_msg.header.frame_id = DEFAULT_MAP_FRAME
        map_msg.header.stamp = rospy.Time.now()

        # info
        map_msg.info.resolution = self.resolution
        map_msg.info.width = self.width
        map_msg.info.height = self.height
        map_msg.data = range(self.width * self.height)

        map_msg.info.origin.position.x = 0
        map_msg.info.origin.position.y = 0

        _grid_1d = np.zeros(len(map_msg.data))
        _grid_1d[:] = -1 # unknown

        _grid_2d = np.reshape(_grid_1d, (self.height, self.width))


        map_msg.data = _grid_1d.astype(np.int8) # grid explicit type casting


        # publishing
        self._map_pub.publish(map_msg)
        self.static_broadcaster()
        self.rate.sleep()