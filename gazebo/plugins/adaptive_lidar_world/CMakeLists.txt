cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

cmake_policy(SET CMP0015 NEW)

project(adaptive_lidar_world)
find_package(gazebo REQUIRED)
include_directories(
    ${GAZEBO_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_BINARY_DIR}/msgs
)
link_directories(${GAZEBO_LIBRARY_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/msgs)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_subdirectory(msgs)

add_library(adaptive_lidar_world SHARED adaptive_lidar_world.cc)
target_link_libraries(adaptive_lidar_world adaptive_lidar_msgs ${GAZEBO_libraries})
add_dependencies(adaptive_lidar_world adaptive_lidar_msgs)
