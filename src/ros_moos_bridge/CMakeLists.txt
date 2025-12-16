cmake_minimum_required(VERSION 3.0.2)
project(ros_moos_bridge)

add_compile_options(-std=c++11)

## === ROS 依赖包 ===
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  sensor_msgs
  uuv_sensor_ros_plugins_msgs
  roslib
)

## === YAML-cpp (用于读取 bridge_config.yaml) ===
find_package(yaml-cpp REQUIRED)

## === MOOS 安装路径 ===
set(MOOS_IVP_PATH /home/xing/moos-ivp)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${MOOS_IVP_PATH}
  ${MOOS_IVP_PATH}/MOOS/MOOSCore/Core/libMOOS/App/include
  ${MOOS_IVP_PATH}/MOOS/MOOSCore/Core/libMOOS/Comms/include
  ${MOOS_IVP_PATH}/MOOS/MOOSCore/Core/libMOOS/Utils/include
  ${MOOS_IVP_PATH}/MOOS/MOOSCore/Core/libMOOS/Thirdparty/getpot/include
  ${MOOS_IVP_PATH}/MOOS/MOOSCore/Core/libMOOS/Thirdparty/PocoBits/include
)

link_directories(
  ${MOOS_IVP_PATH}/build/MOOS/MOOSCore/lib
)

## === ROS 导出包信息 ===
catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    nav_msgs
    sensor_msgs
    uuv_sensor_ros_plugins_msgs
    roslib
)

## === 可执行程序 ===
add_executable(bridge_node
  src/bridge_node.cpp
  src/RosMoosBridge.cpp
)

target_link_libraries(bridge_node
  ${catkin_LIBRARIES}
  yaml-cpp
  MOOS
  pthread
)
