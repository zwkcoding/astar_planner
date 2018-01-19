cmake_minimum_required(VERSION 2.8.3)
project(astar_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_ros
  pcl_conversions
  tf
  opt_utils
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  LIBRARIES hybrid_astar
  CATKIN_DEPENDS roscpp std_msgs tf opt_utils
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(hybrid_astar
  src/astar_search.cpp
  src/search_info_ros.cpp
  src/astar_util.cpp)

target_link_libraries(hybrid_astar
  ${catkin_LIBRARIES})

add_executable(autonav
  src/autonav.cpp)
target_link_libraries(autonav
  ${catkin_LIBRARIES}
  hybrid_astar)
