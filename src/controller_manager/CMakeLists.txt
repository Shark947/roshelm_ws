cmake_minimum_required(VERSION 3.0.2)
project(controller_manager)

add_compile_options(-std=c++14)

# ---------- 依赖 ----------
find_package(catkin REQUIRED COMPONENTS
    pluginlib
    roscpp
    std_msgs
    common_tools
    common_msgs
)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS pluginlib roscpp std_msgs common_tools common_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

# ---------- 插件/核心库 ----------
add_library(${PROJECT_NAME} SHARED
  src/controller_manager.cpp
  src/pid_controller.cpp
  # src/ladrc_controller.cpp
  # 其他控制器源文件按需添加
)

target_link_libraries(${PROJECT_NAME}
  yaml-cpp
  ${catkin_LIBRARIES}
)

# ---------- 主控节点 ----------
add_executable(controller_manager_node src/controller_manager_node.cpp)
target_link_libraries(controller_manager_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  yaml-cpp
)
install(TARGETS controller_manager_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# ---------- 安装 ----------
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# 插件注册描述文件
install(FILES plugin/controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/plugin
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
