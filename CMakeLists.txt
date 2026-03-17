cmake_minimum_required(VERSION 3.0.2)
project(hedge_heading_to_master)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  marvelmind_nav
)

add_message_files(
  FILES
  DroneHeadingDistances.msg
  FleetHeadingDistances.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime marvelmind_nav
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/hedge_heading_distances_node.py
  scripts/dummy_hedge_publisher.py
  scripts/setup_marvelmind_env.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)
