cmake_minimum_required(VERSION 2.8.3)
project(otter_slam)

find_package(catkin REQUIRED roscpp sensor_msgs)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(laser_filter_node src/laser_filter.cpp)
target_link_libraries(laser_filter_node  ${catkin_LIBRARIES})
add_dependencies(laser_filter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Add custom (non compiling) targets so other files show up in QT Creator's project view.
file(GLOB_RECURSE EXTRA_FILES */*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})

