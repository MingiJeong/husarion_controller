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
from enum import Enum

# constants
MAP_FREQ = 10 # Hz
TRANSFORM_DURATION = 3 # sec

DEFAULT_MAP_TOPIC = "map"
DEFAULT_SCAN_TOPIC = 'scan'

DEFAULT_MAP_FRAME = "map"
DEFAULT_ODOM_FRAME = "odom"
DEFAULT_LASER_FRAME = "laser"


class MapLaserUpdate(Enum):
    not_received = 0
    received = 1


class Mapper():
    def __init__(self):
        """ constructor """
        self.rate = rospy.Rate(MAP_FREQ)

        # publisher and subscriber
        self._map_pub = rospy.Publisher(DEFAULT_MAP_TOPIC, OccupancyGrid, queue_size=1)
        self._laser_sub = rospy.Subscriber(DEFAULT_SCAN_TOPIC, LaserScan, self._laser_callback, queue_size=1)
        self._tf_listener = tf.TransformListener()

        # static data of the map
        self.map_origin_x = 0
        self.map_origin_y = 0
        self.resolution = rospy.get_param("map_resolution", 0.05) # m/cell
        self.width = rospy.get_param("width", 200) # cell ==> max_col
        self.height = rospy.get_param("hegiht", 200) # cell ==> max_row
        self.transform_data = (- self.resolution * self.width / 2, 
                               - self.resolution * self.height / 2,
                               0, 0, 0)  # transformed translation (x,y), rotation (r,p,y)

        self.map_max_x = self.width * self.resolution
        self.map_max_y = self.width * self.resolution
        self.map_min_x = self.map_origin_x
        self.map_min_y = self.map_origin_y

        # laser scanner data by ROSBOT
        self.laser_ang_min = None # rad
        self.laser_ang_max = None # rad
        self.laser_ang_inc = None # rad
        self.range_min = None # m
        self.range_max = None # m
        self.laser_data_update = MapLaserUpdate.received


    def _update_laser_data(self):
        """ initial update of the laser msg when booting up """
        laser_msg = rospy.wait_for_message(DEFAULT_SCAN_TOPIC, LaserScan)
        self.laser_ang_min = laser_msg.angle_min
        self.laser_ang_max = laser_msg.angle_max
        self.laser_angle_inc = laser_msg.angle_increment
        self.range_min = laser_msg.range_min
        self.range_max = laser_msg.range_max
        self.laser_data_update = MapLaserUpdate.received


    def _laser_callback(self, laser_msg):

        # process laser data once we receive initial data
        if self.laser_data_update == MapLaserUpdate.received:
            
            map_T_scan = self.get_transform_scan_to_map()
            map_T_odom = self.get_transform_odom_to_map()

            rospy.loginfo("transformation working well {} {}".format(map_T_scan, map_T_odom))

            # do something for mapping
            self.build_map()



    def colrow_to_xy(self, j,i):
        """
        convert column and row index into x,y coordinate (m)
        
        Arguments:
            j: column
            i: row
        """
        x = j * self.resolution + self.map_origin_x
        y = i * self.resolution + self.map_origin_y

        return x,y


    def xy_to_colrow(self, x,y):
        """
        convert x,y coordinate (m) into column and row index (int)
        
        Arguments:
            x: coord_x (m)
            y: coord_y (m)
        """

        i = int((y - self.map_origin_y) // self.resolution) # row
        j = int((x - self.map_origin_x) // self.resolution) # col

        return j, i

    # TODO: convert laser scan data into map frame (i,j) transformation

    def check_inside_map(self, i, j):
        """
        check the given col, row index is bounded by the maximum map area
        """
        if i >= 0 and j>= 0:
            if j <= self.width and i <= self.height:
                return True



    def get_transform_scan_to_map(self):
        """
        obtain transformation matrix between laser and map
        """
        try:
            self._tf_listener.waitForTransform(DEFAULT_MAP_FRAME,
                                            DEFAULT_LASER_FRAME, 
                                            rospy.Time(0), 
                                            rospy.Duration(TRANSFORM_DURATION))

            (trans, rot) = self._tf_listener.lookupTransform(DEFAULT_MAP_FRAME,
                                                            DEFAULT_LASER_FRAME,
                                                            rospy.Time(0))
            t = tf.transformations.translation_matrix(trans)
            R = tf.transformations.quaternion_matrix(rot) 

            return np.dot(t, R)

        except:
            rospy.loginfo("transformation not working from %s to %s" 
                            %(DEFAULT_LASER_FRAME, DEFAULT_MAP_FRAME))


    def get_transform_odom_to_map(self):
        """
        obtain transformation matrix between odom and map
        """
        try:
            self._tf_listener.waitForTransform(DEFAULT_MAP_FRAME,
                                            DEFAULT_ODOM_FRAME, 
                                            rospy.Time(0), 
                                            rospy.Duration(TRANSFORM_DURATION))

            (trans, rot) = self._tf_listener.lookupTransform(DEFAULT_MAP_FRAME,
                                                            DEFAULT_ODOM_FRAME,
                                                            rospy.Time(0))
            t = tf.transformations.translation_matrix(trans)
            R = tf.transformations.quaternion_matrix(rot) 

            return np.dot(t, R)

        except:
            rospy.loginfo("transformation not working from %s to %s" 
                            %(DEFAULT_ODOM_FRAME, DEFAULT_MAP_FRAME))


    def static_broadcaster(self):
        """
        static transform broadcaster between odom and map frame
        """
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
        """
        building and filling the Occupancy grid map
        """
        map_msg = OccupancyGrid()

        # header
        map_msg.header.frame_id = DEFAULT_MAP_FRAME
        map_msg.header.stamp = rospy.Time.now()

        # info
        map_msg.info.resolution = self.resolution
        map_msg.info.width = self.width
        map_msg.info.height = self.height
        map_msg.data = range(self.width * self.height)

        map_msg.info.origin.position.x = self.map_origin_x
        map_msg.info.origin.position.y = self.map_origin_y

        _grid_1d = np.zeros(len(map_msg.data))
        _grid_1d[:] = -1 # unknown

        _grid_2d = np.reshape(_grid_1d, (self.height, self.width))


        # TODO: grid change
        map_msg.data = _grid_1d.astype(np.uint8) # grid explicit type casting

        # publishing
        self._map_pub.publish(map_msg)
        self.static_broadcaster()
        self.rate.sleep()

