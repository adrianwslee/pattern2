cmake_minimum_required(VERSION 2.8.3)
project(excavator_core)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs
)

install(PROGRAMS
  scripts/start_node
  scripts/test_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
