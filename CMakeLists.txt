cmake_minimum_required(VERSION 3.0.2)
project(husarion_controller)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tf
)

catkin_python_setup()


## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES husarion-controller
   CATKIN_DEPENDS rospy tf
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
   nodes/main_controller
   nodes/mapper
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


