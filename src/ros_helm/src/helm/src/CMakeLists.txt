#--------------------------------------------------------
# The CMakeLists.txt for:                        pHelmIvP
# Author(s):                                Mike Benjamin
#--------------------------------------------------------

# Set System Specific Libraries
if (${WIN32})
  SET(SYSTEM_LIBS
    wsock32)
else (${WIN32})
  SET(SYSTEM_LIBS
    dl
    m
    pthread)
endif (${WIN32})

SET(SRC
  HelmApp.cpp
  HelmCheck.cpp
  HelmIvP.cpp
  HelmEngine.cpp
)

set(ROSHLEM_DEFAULT_CONFIG_PATH
    "${ROS_HELM_SOURCE_DIR}/src/helm/config/startHelm.yaml")
file(TO_CMAKE_PATH "${ROSHLEM_DEFAULT_CONFIG_PATH}"
     ROSHLEM_DEFAULT_CONFIG_PATH)

add_library(helm_core ${SRC})
target_compile_features(helm_core PUBLIC cxx_std_17)

target_link_libraries(helm_core
  ${MOOS_LIBRARIES}
  ${MOOSGeodesy_LIBRARIES}
  helmivp
  dep_behaviors
  behaviors-marine
  geodaid
  contacts
  behaviors-colregs
  ufield
  behaviors
  bhvutil	
  turngeo
  ivpbuild 
  ivpcore
  ivpsolve 
  geometry
  apputil
  mbutil
  logic
  genutil
  ${SYSTEM_LIBS})
target_compile_definitions(helm_core PUBLIC
  DEFAULT_CONFIG_PATH="${ROSHLEM_DEFAULT_CONFIG_PATH}")

ADD_EXECUTABLE(pHelmIvP main.cpp)

target_compile_features(pHelmIvP PRIVATE cxx_std_17)

TARGET_LINK_LIBRARIES(pHelmIvP
  helm_core)

target_compile_definitions(pHelmIvP PRIVATE
  DEFAULT_CONFIG_PATH="${ROSHLEM_DEFAULT_CONFIG_PATH}")


