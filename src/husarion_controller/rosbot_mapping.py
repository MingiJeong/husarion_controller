#!/usr/bin/env python

"""
    rosbot_mapping.py

    purpose: 
    - Occupancy grid mapping by rosbot
"""

# import relevant libraries
from husarion_controller.rosbot_setup import DEFAULT_CMD_VEL
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

# import python modules
import numpy as np
import math

# constants
MAP_FREQ = 10 # Hz

DEFAULT_MAP_TOPIC = "map"
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


    def _laser_callback(self, laser_msg):
        # do something for mapping
        self.build_map()


    def build_map(self):
        map_msg = OccupancyGrid()

        # header
        map_msg.header.frame_id = "map"
        map_msg.header.stamp = rospy.Time.now()

        # info
        map_msg.info.resolution = self.resolution
        map_msg.info.width = self.width
        map_msg.info.height = self.height
        map_msg.data = range(self.width * self.height)

        map_msg.info.origin.position.x = -2.5
        map_msg.info.origin.position.y = -2.5

        _grid_1d = np.zeros(len(map_msg.data))
        _grid_1d[:] = -1 # unknown

        _grid_2d = np.reshape(_grid_1d, (self.height, self.width))

        map_msg.data = _grid_1d.astype(np.uint8)

        self._map_pub.publish(map_msg)
        self.rate.sleep()
