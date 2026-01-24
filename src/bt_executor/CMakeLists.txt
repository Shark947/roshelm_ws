cmake_minimum_required(VERSION 3.0.2)
project(bt_executor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  behaviortree_cpp_v3
  common_msgs
  docking_optical_msgs
  geometry_msgs
  ros_helm
  roscpp
  roslib
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bt_executor_core
  CATKIN_DEPENDS behaviortree_cpp_v3 common_msgs docking_optical_msgs geometry_msgs ros_helm roscpp roslib std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(bt_executor_core
  src/bt_executor.cpp
  src/helm_adapter.cpp
  src/docking_phase_manager.cpp
  src/utils.cpp
  src/nodes/actions.cpp
  src/nodes/conditions.cpp
)

target_link_libraries(bt_executor_core
  ${catkin_LIBRARIES}
)

target_compile_features(bt_executor_core PUBLIC cxx_std_17)

add_executable(bt_executor_node src/bt_executor_node.cpp)

target_link_libraries(bt_executor_node
  bt_executor_core
  ${catkin_LIBRARIES}
)

target_compile_features(bt_executor_node PUBLIC cxx_std_17)

add_dependencies(bt_executor_core ${catkin_EXPORTED_TARGETS})
add_dependencies(bt_executor_node ${catkin_EXPORTED_TARGETS})

install(TARGETS bt_executor_core bt_executor_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY behavior_trees config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(test_bt_executor
    test/test_bt_executor.cpp
  )
  if(TARGET test_bt_executor)
    target_link_libraries(test_bt_executor bt_executor_core ${catkin_LIBRARIES})
    target_compile_features(test_bt_executor PUBLIC cxx_std_17)
  endif()
endif()
