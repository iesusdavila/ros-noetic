cmake_minimum_required(VERSION 3.0.2)
project(staubli_tx2_60_support)

find_package(catkin REQUIRED)

catkin_package()

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(tests/roslaunch_test_tx2_60.xml)
  roslaunch_add_file_check(tests/roslaunch_test_tx2_60l.xml)
endif()

install(DIRECTORY config launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
