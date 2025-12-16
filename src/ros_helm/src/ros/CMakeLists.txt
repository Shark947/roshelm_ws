cmake_minimum_required(VERSION 3.10)
project(ros_helm_node)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  common_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/src/ros/include
  ${CMAKE_SOURCE_DIR}/src/helm/include
  ${CMAKE_SOURCE_DIR}/src/helm/tools
)

add_executable(ros_helm_node
  src/ros_node_main.cpp
  src/RosBridge.cpp
  src/RosConfigLoader.cpp)

target_link_libraries(ros_helm_node
  helm_core
  ${catkin_LIBRARIES})

install(TARGETS ros_helm_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
