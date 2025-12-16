cmake_minimum_required(VERSION 3.0.2)
project(common_tools)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# header-only，无需add_library/target_link_libraries
