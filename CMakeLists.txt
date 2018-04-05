cmake_minimum_required(VERSION 2.8.3)
project(astar_planner)

#if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
#    set(CMAKE_BUILD_TYPE Release)
#endif()

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        pcl_ros
        pcl_conversions
        tf
        opt_utils
        internal_grid_map
        visualization_msgs
        control_msgs
        grid_map_ros
        car_model
        iv_explore_msgs
        )
find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
        LIBRARIES hybrid_astar
        CATKIN_DEPENDS roscpp std_msgs tf opt_utils internal_grid_map visualization_msgs car_model iv_explore_msgs
        DEPENDS OpenCV
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
        include
        ${OpenCV_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
        src/astar_search.cpp
        src/search_info_ros.cpp
        src/astar_util.cpp)

set(linked_libs
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        )
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES})

add_executable(autonav_node
        src/autonav_node.cpp)
target_link_libraries(autonav_node
        ${linked_libs})

add_executable(astar_server_node
        src/astar_action_server_node.cpp)
target_link_libraries(astar_server_node
        ${linked_libs})

add_executable(get_plan_server_node
        src/get_plan_server_node.cpp)
target_link_libraries(get_plan_server_node
        ${linked_libs})

add_executable(get_plan_client_node
        tests/get_plan_client_node.cpp)
target_link_libraries(get_plan_client_node
        ${linked_libs})