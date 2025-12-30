add_executable(ros_helm_node
  src/ros_node_main.cpp
  src/RosBridge.cpp
  src/RosConfigLoader.cpp)

target_include_directories(ros_helm_node PRIVATE
  ${catkin_INCLUDE_DIRS}
  ${ROS_HELM_SOURCE_DIR}/src/ros/include
  ${ROS_HELM_SOURCE_DIR}/src/helm/include
  ${ROS_HELM_SOURCE_DIR}/src/helm/tools)

target_link_libraries(ros_helm_node
  helm_core
  ${catkin_LIBRARIES})

add_executable(ros_nav_publish
  src/RosNavPublish.cpp)

target_include_directories(ros_nav_publish PRIVATE
  ${catkin_INCLUDE_DIRS})

target_link_libraries(ros_nav_publish
  ${catkin_LIBRARIES})

install(TARGETS ros_helm_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS ros_nav_publish
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY ${ROS_HELM_SOURCE_DIR}/config/ros
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
