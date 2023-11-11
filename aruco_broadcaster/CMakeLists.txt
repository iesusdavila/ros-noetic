cmake_minimum_required(VERSION 3.0.2)
project(aruco_broadcaster)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    message_generation
    geometry_msgs
    genmsg
    std_msgs
    aruco
    aruco_msgs
    tf2_ros
    tf2
    tf2_geometry_msgs
    roscpp
    eigen_conversions)

find_package(Eigen3 REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################
## Generate services in the 'srv' folder
add_service_files(
  FILES
  getMarkerTf.srv
)
## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  ${EIGEN3_INCLUDE_DIR}
  #LIBRARIES aruco_mapping
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs tf2 tf2_ros tf2_geometry_msgs std_msgs message_runtime geometry_msgs
  DEPENDS EIGEN3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# deprecated
# add_executable(aruco_broadcaster src/aruco_mapping.cpp)
# target_link_libraries(aruco_broadcaster2 ${catkin_LIBRARIES})
add_executable(aruco_broadcaster src/aruco_broadcaster.cpp)
target_link_libraries(aruco_broadcaster ${catkin_LIBRARIES})


## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(aruco_broadcaster ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(aruco_broadcaster2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

