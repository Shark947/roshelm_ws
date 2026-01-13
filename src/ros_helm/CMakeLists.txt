cmake_minimum_required(VERSION 3.14)
project(ros_helm LANGUAGES CXX)

set(ROS_HELM_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  common_msgs
  roscpp
  roslib
  std_msgs
  tf2
  tf2_geometry_msgs
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# --------------------------------------------------------------
# Default configuration path
# --------------------------------------------------------------
set(DEFAULT_CONFIG_PATH "${ROS_HELM_SOURCE_DIR}/config/helm/startHelm.yaml")
file(TO_CMAKE_PATH "${DEFAULT_CONFIG_PATH}" DEFAULT_CONFIG_PATH)
add_compile_definitions(DEFAULT_CONFIG_PATH=\"${DEFAULT_CONFIG_PATH}\")

# --------------------------------------------------------------
# Paths
# --------------------------------------------------------------
set(IVP_SOURCE_DIR ${ROS_HELM_SOURCE_DIR}/src/ivp/src)

# --------------------------------------------------------------
# IvP Libraries (directories)
# --------------------------------------------------------------
set(IVP_LIBS
  lib_behaviors
  lib_bhvutil
  lib_behaviors-marine
  lib_genutil
  lib_geometry
  lib_behaviors-colregs
  lib_polar
  lib_helmivp
  lib_ivpbuild
  lib_ivpsolve
  lib_logic
  lib_ivpcore
  lib_evalutil
  lib_encounters
  lib_contacts
  lib_obstacles
  lib_realm
  lib_ufield
  lib_logutils
  lib_mbutil
  lib_manifest
  lib_turngeo
  lib_survey
  lib_dep_behaviors
)

# --------------------------------------------------------------
# Verify directories exist
# --------------------------------------------------------------
message(STATUS "IVP source dir = ${IVP_SOURCE_DIR}")
foreach(lib ${IVP_LIBS})
  set(dir_path ${IVP_SOURCE_DIR}/${lib})
  if(NOT IS_DIRECTORY ${dir_path})
    message(FATAL_ERROR "Missing required IVP directory: ${dir_path}")
  endif()
endforeach()

# --------------------------------------------------------------
# Add include dirs
# --------------------------------------------------------------
set(IVP_INCLUDE_DIRS
  ${ROS_HELM_SOURCE_DIR}/src
  ${ROS_HELM_SOURCE_DIR}/src/helm/include
  ${ROS_HELM_SOURCE_DIR}/src/helm/tools
  ${ROS_HELM_SOURCE_DIR}/src/ros/include
  ${catkin_INCLUDE_DIRS}
)

foreach(lib ${IVP_LIBS})
  list(APPEND IVP_INCLUDE_DIRS ${IVP_SOURCE_DIR}/${lib})
endforeach()

set(ROS_HELM_PUBLIC_INCLUDE_DIRS
  ${ROS_HELM_SOURCE_DIR}/src
  ${ROS_HELM_SOURCE_DIR}/src/helm/include
  ${ROS_HELM_SOURCE_DIR}/src/helm/tools
  ${ROS_HELM_SOURCE_DIR}/src/ros/include
)
foreach(lib ${IVP_LIBS})
  list(APPEND ROS_HELM_PUBLIC_INCLUDE_DIRS ${IVP_SOURCE_DIR}/${lib})
endforeach()

include_directories(${IVP_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS
    ${ROS_HELM_PUBLIC_INCLUDE_DIRS}
  LIBRARIES helm_core ros_helm_bridge
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    common_msgs
    roscpp
    roslib
    std_msgs
    tf2
    tf2_geometry_msgs
)

# --------------------------------------------------------------
# Add all IVP subdirectories
# --------------------------------------------------------------
foreach(lib ${IVP_LIBS})
  add_subdirectory(${IVP_SOURCE_DIR}/${lib} ${CMAKE_BINARY_DIR}/ivp/${lib})
endforeach()

add_subdirectory(src/helm/test ${CMAKE_BINARY_DIR}/helm_test)
add_subdirectory(src/helm/src ${CMAKE_BINARY_DIR}/helm_app)
add_subdirectory(src/ros ${CMAKE_BINARY_DIR}/ros_node)
