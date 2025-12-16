cmake_minimum_required(VERSION 3.0.2)
project(variable_extractor)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  sensor_msgs
  pluginlib
  message_generation
  common_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES variable_extractor
  CATKIN_DEPENDS
    roscpp
    std_msgs
    nav_msgs
    sensor_msgs
    pluginlib
    message_runtime
    common_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# ---------------------- 插件库 ----------------------
add_library(variable_extractor
  src/variable_extractor.cpp
  # 多个插件实现写在此
  # src/other_variable_extractor.cpp
)
add_dependencies(variable_extractor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(variable_extractor
  ${catkin_LIBRARIES}
)

# ---------------------- 主节点 ----------------------
add_executable(variable_extractor_node
  src/variable_extractor_node.cpp
)
add_dependencies(variable_extractor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(variable_extractor_node
  variable_extractor  
  ${catkin_LIBRARIES}
)

# ---------------------- 安装 ----------------------
install(
  TARGETS variable_extractor variable_extractor_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  FILES plugin/variable_extractor_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/plugin
)
