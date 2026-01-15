add_library(ros_helm_bridge
  src/HelmVariableInjector.cpp
  src/RosBridge.cpp
  src/RosCommandPublisher.cpp
  src/RosConfigLoader.cpp
  src/RosNavPublisher.cpp)

target_include_directories(ros_helm_bridge PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${ROS_HELM_SOURCE_DIR}/src/ros/include
  ${ROS_HELM_SOURCE_DIR}/src/helm/include
  ${ROS_HELM_SOURCE_DIR}/src/helm/tools)

target_link_libraries(ros_helm_bridge
  helm_core
  ${catkin_LIBRARIES})

add_executable(ros_helm_node
  src/ros_node_main.cpp)

target_link_libraries(ros_helm_node
  ros_helm_bridge)

install(TARGETS ros_helm_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS ros_helm_bridge
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY ${ROS_HELM_SOURCE_DIR}/config/ros
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
