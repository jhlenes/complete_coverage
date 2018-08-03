cmake_minimum_required(VERSION 2.8.3)
project(otter_coverage)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  otter_navigation
  roscpp
  tf2
  tf2_ros
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


## Add nodes
add_library(coverage src/coverage.cpp)
target_link_libraries(coverage ${catkin_LIBRARIES})
add_executable(coverage_node src/coverage_node.cpp)
target_link_libraries(coverage_node coverage)

add_library(guidance src/guidance.cpp)
target_link_libraries(guidance ${catkin_LIBRARIES})
add_executable(guidance_node src/guidance_node.cpp)
target_link_libraries(guidance_node guidance)

add_library(map_processing src/map_processing.cpp)
target_link_libraries(map_processing ${catkin_LIBRARIES})
add_executable(map_processing_node src/map_processing_node.cpp)
target_link_libraries(map_processing_node map_processing)

## Add custom (non compiling) targets so other files show up in QT Creator's project view.
file(GLOB_RECURSE EXTRA_FILES */*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})

