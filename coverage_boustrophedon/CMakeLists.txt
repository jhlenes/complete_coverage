cmake_minimum_required(VERSION 2.8.3)
project(coverage_boustrophedon)

add_compile_options(-std=c++14)
find_package(catkin REQUIRED COMPONENTS roscpp tf2 tf2_ros std_msgs message_generation geometry_msgs message_runtime)

add_message_files(
  FILES
  DubinInput.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)


## Add nodes
add_executable(coverage_node
  include/coverage/a_star.h
  include/coverage/coverage.h
  include/coverage/partition.h
  include/simple_dubins_path/simple_dubins_path.h
  src/simple_dubins_path.cpp
  src/coverage_node.cpp
  src/coverage.cpp
  src/partition.cpp
)
add_dependencies(coverage_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(coverage_node
  ${catkin_LIBRARIES}
)

add_executable(simple_dubins_path_node
  include/simple_dubins_path/simple_dubins_path.h
  src/simple_dubins_path_node.cpp
  src/simple_dubins_path.cpp
)
add_dependencies(simple_dubins_path_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_dubins_path_node
  ${catkin_LIBRARIES}
)

## Add custom (non compiling) targets so other files show up in QT Creator's project view.
file(GLOB_RECURSE EXTRA_FILES */*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})

