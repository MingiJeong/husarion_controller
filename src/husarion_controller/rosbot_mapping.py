#!/usr/bin/env python

"""
    rosbot_mapping.py

    purpose: 
    - Occupancy grid mapping by rosbot
"""

# import relevant libraries
from numpy.core.numeric import True_
import rospy
import tf
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from tf.transformations import euler_from_quaternion, quaternion_from_euler


# import python modules
import numpy as np
import math
import sys
from enum import Enum

# custom module
from husarion_controller import constant

# constants
MAP_FREQ = constant.MAP_FREQ
TRANSFORM_DURATION = constant.TRANSFORM_DURATION

DEFAULT_MAP_TOPIC = constant.DEFAULT_MAP_TOPIC
DEFAULT_SCAN_TOPIC = constant.DEFAULT_SCAN_TOPIC

DEFAULT_MAP_FRAME = constant.DEFAULT_MAP_FRAME
DEFAULT_ODOM_FRAME = constant.DEFAULT_ODOM_FRAME
DEFAULT_LASER_FRAME = constant.DEFAULT_LASER_FRAME
DEFAULT_BASE_LINK_FRAME = constant.DEFAULT_BASE_LINK_FRAME


class MapLaserUpdate(Enum):
    not_received = 0
    received = 1


class Mapper():
    def __init__(self):
        """ constructor """

        # essential properties
        self.rate = rospy.Rate(MAP_FREQ)

        self.robot_heading_wrt_map = None
        self.robot_pose_x_wrt_map = None
        self.robot_pose_y_wrt_map = None

        self.map_T_scan = None
        self.map_T_odom = None
        self.map_T_base = None

        self._grid_1d = None
        self._grid_2d = None

        # publisher and subscriber
        self._map_pub = rospy.Publisher(DEFAULT_MAP_TOPIC, OccupancyGrid, queue_size=1)
        self._laser_sub = rospy.Subscriber(DEFAULT_SCAN_TOPIC, LaserScan, self._laser_callback, queue_size=1)
        self._tf_listener = tf.TransformListener()

        # static data of the map
        self.map_origin_x = 0
        self.map_origin_y = 0
        self.resolution = rospy.get_param("map_resolution", 0.05) # m/cell
        self.width = rospy.get_param("width", 200) # cell ==> max_col
        self.height = rospy.get_param("height", 200) # cell ==> max_row
        self.transform_data = (- self.resolution * self.width / 2, 
                               - self.resolution * self.height / 2,
                               0, 0, 0)  # transformed translation (x,y), rotation (r,p,y)

        self.map_max_x = self.width * self.resolution
        self.map_max_y = self.height * self.resolution
        self.map_min_x = self.map_origin_x
        self.map_min_y = self.map_origin_y


        self.map_data_length = self.width * self.height
        self.map_msg = None
        self.ready_to_publish = False # flipped condition due to multi threading

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


    def map_initializer(self):
        """
        initializing the map data and flip
        """
        if self._grid_2d is None:
            _grid_1d = np.zeros(self.map_data_length)
            _grid_1d[:] = -1 # unknown

            self._grid_2d = np.reshape(_grid_1d, (self.height, self.width))

        self.flip_map_row()

        self.ready_to_publish = False


    def _laser_callback(self, laser_msg):
        """
        process laser data once we receive initial data
        """
        
        if self.laser_data_update == MapLaserUpdate.received:
            # pre-requisites

            global transformed_coord, ray_cast_index
            transformed_coord = []
            ray_cast_index = []

            # transformation matrix 
            self.map_T_scan = self.get_transform_scan_to_map() # scan wrt map
            self.map_T_odom = self.get_transform_odom_to_map() # odom wrt map
            self.map_T_base = self.get_transform_baselink_to_map() # baselink wrt map

            """ check a map is good to be built """
            if self.check_mapping_condition():
                # map to be built
                self.map_initializer()
    
                # 1) robot position on the map (col, row index)
                robot_col, robot_row = self.xy_to_colrow(self.robot_pose_x_wrt_map,
                                                         self.robot_pose_y_wrt_map)

                # 2) looping over each laser scan data on the map (col, row index)
                for i, data in enumerate(laser_msg.ranges):
                    # 2-1) valid scan converted to col, row index
                    self.process_scanpoint_to_vec(i, data, laser_msg)
                    self.raycast_build(transformed_coord[i])

                    # 2-2) raycast by line find algorithm
                    bresenham_result = self.bresenham_algorithm(robot_col, robot_row,
                                                ray_cast_index[i][0], ray_cast_index[i][1])

                    # 2-3) grid filling based on the line finding algorithm
                    self.fill_grid_map(bresenham_result)


                # 3) publish map
                self.build_map()

    def fill_grid_map(self, bresenham_result):
        
        # edge case
        if bresenham_result is None:
            return

        # initialization
        points_in_line, is_steep = bresenham_result

        for k, point in enumerate(points_in_line):
            if self.check_inside_map(point[1], point[0]):

                if k < len(points_in_line) - 1:
                    if self.check_inside_map(point[1], point[0]):
                        continue # skip as already occupied
                    self._grid_2d[point[1], point[0]] = 0 # (col, row) -> (row, col)
                
                else: # k == len(points_in_line) - 1:
                    self._grid_2d[point[1], point[0]] = 100 # (col, row) -> (row, col)


    def bresenham_algorithm(self, col_1, row_1, col_2, row_2):
        """
        reference: 
        https://www.youtube.com/watch?v=hr9e2C_0RrQ
        http://www.roguebasin.com/index.php/Bresenham%27s_Line_Algorithm#Python

        start: col_1, row_1
        end: col_2, row_2
        """

        # edge case
        if not np.isfinite(col_2) or not np.isfinite(row_2):
            return None

        # valid laser scan check for being inside the map
        if self.check_inside_map(col_2, row_2):

            dx = col_2 - col_1
            dy = row_2 - row_1

            # steep
            is_steep = abs(dy) > abs(dx)

            if is_steep:
                # swap
                col_1, row_1 = row_1, col_1
                col_2, row_2 = row_2, col_2

            swapped = False
            if col_1 > col_2:
                col_1, col_2 = col_2, col_1
                row_1, row_2 = row_2, row_1
                swapped = True

            dx = col_2 - col_1
            dy = row_2 - row_1

            # Calculate error
            error = int(dx / 2.0)
            ystep = 1 if row_1 < row_2 else -1

            y = row_1
            points_in_line = []
            for x in range(col_1, col_2 + 1):
                coord = (y, x) if is_steep else (x, y)

                points_in_line.append(coord)
                error -= abs(dy)
                if error < 0:
                    y += ystep
                    error += dx

            if swapped:
                points_in_line.reverse()

            return (points_in_line, is_steep)


    def raycast_build(self, scan_xy):

        if np.isfinite(scan_xy[0]) and np.isfinite(scan_xy[1]):
            scan_col, scan_row = self.xy_to_colrow(scan_xy[0],
                                                   scan_xy[1])
            ray_cast_index.append([scan_col, scan_row])
        
        else:
            ray_cast_index.append([np.nan, np.nan])            


    def process_scanpoint_to_vec(self, i, data, laser_msg):
        """
        convert the scan point data (distance) into 4 x 1 vector to multiply with map_T_scan matrix
        """
        
        if np.isfinite(data): # valid data
            angle = laser_msg.angle_min + i * laser_msg.angle_increment
            x = data * np.cos(angle)
            y = data * np.sin(angle)

            transformed_coord.append(self.map_T_scan.dot(np.transpose(np.array([x,y,0,1]))))

        else: # inf data measurement by laser
            transformed_coord.append(np.array([np.nan, np.nan, np.nan, np.nan]))
        

    def check_mapping_condition(self):
        """
        check the pre-requisite valid mapping conditions are met
        """
        if self.robot_heading_wrt_map is not None and \
           self.robot_pose_x_wrt_map is not None and \
           self.robot_pose_y_wrt_map is not None and \
           self.map_T_scan is not None and \
           self.map_T_base is not None and \
           self.map_T_odom is not None:
            return True

        return False

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


    def check_map_occupied(self, j, i):
        """
        check the given col, row index is occupied in the map area

        Arguments:
            j: col
            i: row
        """

        if self._grid_2d[j,i] == 100:
            return True


    def check_inside_map(self, j, i):
        """
        check the given col, row index is bounded by the maximum map area

        Arguments:
            j: col
            i: row
            
        """
        if i >= 0 and j>= 0:
            if j <= self.width-1 and i <= self.height-1:
                return True

    def transform_ray_to_map(self, laser_msg):
        pass


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


    def get_transform_baselink_to_map(self):
        """
        obtain transformation matrix between odom and map
        """
        try:
            self._tf_listener.waitForTransform(DEFAULT_MAP_FRAME,
                                            DEFAULT_BASE_LINK_FRAME, 
                                            rospy.Time(0), 
                                            rospy.Duration(TRANSFORM_DURATION))

            (trans, rot) = self._tf_listener.lookupTransform(DEFAULT_MAP_FRAME,
                                                            DEFAULT_BASE_LINK_FRAME,
                                                            rospy.Time(0))
        
            t = tf.transformations.translation_matrix(trans)
            R = tf.transformations.quaternion_matrix(rot) 

            # acqurie robot pose wrt "MAP"
            self.robot_pose_x_wrt_map = trans[0] # x
            self.robot_pose_y_wrt_map = trans[1] # y

            (_,_,yaw) = euler_from_quaternion(rot)
            self.robot_heading_wrt_map = yaw # theta

            return np.dot(t, R)

        except:
            rospy.loginfo("transformation not working from %s to %s" 
                            %(DEFAULT_BASE_LINK_FRAME, DEFAULT_MAP_FRAME))


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

    def flip_map_row(self):
        self._grid_2d = np.flipud(self._grid_2d)

    def flatten_grid_2d(self):
        self._grid_1d = self._grid_2d.flatten()


    def build_map(self):
        """
        building and filling the Occupancy grid map
        """
        self.map_msg = OccupancyGrid()

        # header
        self.map_msg.header.frame_id = DEFAULT_MAP_FRAME
        self.map_msg.header.stamp = rospy.Time.now()

        # info
        self.map_msg.info.resolution = self.resolution
        self.map_msg.info.width = self.width
        self.map_msg.info.height = self.height
        self.map_msg.data = range(self.width * self.height)

        self.map_msg.info.origin.position.x = self.map_origin_x
        self.map_msg.info.origin.position.y = self.map_origin_y

        # flip after all processed
        self.flip_map_row()

        self.ready_to_publish = True

    def publsih_map(self):
        # edge case check
        if self._grid_2d is not None and \
            self.map_msg is not None:

            if self.ready_to_publish:
                # publish purpose flip
                tmp = np.flipud(self._grid_2d).flatten()
                self.map_msg.data = tmp.astype(np.int8) # grid explicit type casting
                
                self._map_pub.publish(self.map_msg)


    def spin(self):
        """ 
        multi-threading enabler for map publisher vs other subscribers
        """
        while not rospy.is_shutdown():
            self.static_broadcaster()
            self.publsih_map()

            self.rate.sleep()
