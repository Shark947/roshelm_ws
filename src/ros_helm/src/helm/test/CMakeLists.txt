cmake_minimum_required(VERSION 3.14)
project(helm_tests LANGUAGES CXX)

set(HELM_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)
get_filename_component(REPO_ROOT ${HELM_ROOT}/.. ABSOLUTE)
set(HELM_INCLUDE_DIRS
    ${HELM_ROOT}/include
    ${HELM_ROOT}/tools)

set(HELM_SOURCES
    ${HELM_ROOT}/src/HelmEngine.cpp
    ${HELM_ROOT}/src/HelmIvP.cpp
    ${HELM_ROOT}/src/HelmApp.cpp
    ${HELM_ROOT}/src/HelmCheck.cpp
    ${HELM_ROOT}/tools/ContactLedger.cpp
    ${HELM_ROOT}/tools/HelmGeodesy.cpp
    ${HELM_ROOT}/tools/HelmMsg.cpp
    ${HELM_ROOT}/tools/HelmUtilityFunctions.cpp)

set(IVP_SOURCE_DIR ${REPO_ROOT}/ivp/src)

set(TEST_IVP_INCLUDE_DIRS ${IVP_SOURCE_DIR})
file(GLOB TEST_IVP_SUBDIRS LIST_DIRECTORIES true
     ${IVP_SOURCE_DIR}/lib_*
     ${IVP_SOURCE_DIR}/app_*)

foreach(dir ${TEST_IVP_SUBDIRS})
  if(IS_DIRECTORY ${dir})
    list(APPEND TEST_IVP_INCLUDE_DIRS ${dir})
  endif()
endforeach()

add_executable(test_ivp test_ivp.cpp)
target_compile_features(test_ivp PRIVATE cxx_std_17)
target_include_directories(test_ivp PRIVATE ${TEST_IVP_INCLUDE_DIRS})

target_link_libraries(test_ivp PRIVATE
  helmivp
  behaviors
  behaviors-marine
  ivpbuild
  logic
  ivpcore
  mbutil
  bhvutil
  genutil
  contacts
  geometry
)

add_executable(test_helm test_helm.cpp)
target_sources(test_helm PRIVATE ${HELM_SOURCES})
target_compile_features(test_helm PRIVATE cxx_std_17)
target_include_directories(test_helm PRIVATE
  ${TEST_IVP_INCLUDE_DIRS}
  ${HELM_INCLUDE_DIRS})
target_link_libraries(test_helm PRIVATE
  helmivp
  behaviors
  behaviors-marine
  behaviors-colregs
  contacts
  genutil
  ivpcore
  ivpbuild
  ivpsolve
  mbutil
  geometry
  turngeo
  evalutil
  encounters
  obstacles
  realm
  ufield
  dep_behaviors
  manifest
  polar
  bhvutil
  pthread
  dl)
