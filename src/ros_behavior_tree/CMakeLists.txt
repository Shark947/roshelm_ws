cmake_minimum_required(VERSION 3.0.2)
project(ros_behavior_tree)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  behaviortree_cpp_v3
  common_msgs
  roscpp
  ros_helm
  roslib
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS behaviortree_cpp_v3 common_msgs roscpp ros_helm roslib std_msgs
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)

add_executable(behavior_tree_node
  src/behavior_tree_node.cpp
  src/bt/register_nodes.cpp
  src/bt_nodes.cpp
  src/helm_interface.cpp
  src/nav_subscriber.cpp
  src/nodes/actions/constant_speed_action.cpp
  src/nodes/actions/dummy_action.cpp
  src/nodes/actions/mission_complete_action.cpp
  src/nodes/actions/waypoint_action.cpp
  src/nodes/conditions/common/deploy_triggered.cpp
  src/nodes/conditions/common/return_triggered.cpp
  src/nodes/conditions/common/speed_triggered.cpp
)

add_dependencies(behavior_tree_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(behavior_tree_node
  ${catkin_LIBRARIES}
  behaviortree_cpp_v3
  helm_core
  zmq
)

add_executable(nav_publisher_node
  src/nav_publisher_node.cpp
)

add_dependencies(nav_publisher_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(nav_publisher_node
  ${catkin_LIBRARIES}
)

install(TARGETS behavior_tree_node nav_publisher_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
