import math

# constants

# MAP NODE
# ----------------------------
MAP_FREQ = 5 # Hz
TRANSFORM_DURATION = 3 # sec
MAPPING_DIST_THRESH = 7 # m

DEFAULT_MAP_TOPIC = "map"
# DEFAULT_SCAN_TOPIC = "scan"
DEFAULT_SCAN_TOPIC = 'base_scan'

DEFAULT_MAP_FRAME = "map"
DEFAULT_ODOM_FRAME = "odom"
# DEFAULT_LASER_FRAME = "laser"
DEFAULT_LASER_FRAME = "base_laser_link"

DEFAULT_BASE_LINK_FRAME = "base_link"
# ----------------------------


# ----------------------------
FREQUENCY = 10 # Hz
LIN_VEL = 0.2 # m/s
ANG_VEL = math.pi/18 # rad/s

DEFAULT_CMD_VEL = 'cmd_vel'
DEFAULT_ODOM = 'odom'
# ----------------------------