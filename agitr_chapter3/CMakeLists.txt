# What version of CMake is needed?
cmake_minimum_required(VERSION 2.8.3)

# The name of this package.
project(agitr_chapter3)

# Find the catkin build system, and any other packages on which we depend.
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs turtlesim)

# Declare our catkin package.
catkin_package()

# Specify locations of header files.
include_directories(include ${catkin_INCLUDE_DIRS})

# Declare the executable, along with its source files.
add_executable(pubvel src/pubvel.cpp)
add_executable(subpose src/subpose.cpp)
add_executable(pubvelsafe src/pubvelsafe.cpp)
add_executable(avoidcollision src/avoidcollision.cpp)

# Specify libraries against which to link.
target_link_libraries(pubvel ${catkin_LIBRARIES})
target_link_libraries(subpose ${catkin_LIBRARIES})
target_link_libraries(pubvelsafe ${catkin_LIBRARIES})
target_link_libraries(avoidcollision ${catkin_LIBRARIES})