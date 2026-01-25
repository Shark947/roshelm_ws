cmake_minimum_required(VERSION 3.0.2)
project(ros_behavior_tree)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  common_msgs
  roscpp
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS common_msgs roscpp std_msgs
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(behavior_tree_node
  src/behavior_tree_node.cpp
  src/bt_nodes.cpp
  src/nav_subscriber.cpp
)

add_dependencies(behavior_tree_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(behavior_tree_node
  ${catkin_LIBRARIES}
)

install(TARGETS behavior_tree_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
