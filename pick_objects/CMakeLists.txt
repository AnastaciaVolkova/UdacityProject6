cmake_minimum_required(VERSION 3.0.2)
project(pick_objects)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  roscpp
  add_markers
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES add_markers_node
  CATKIN_DEPENDS roscpp std_msgs 
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pick_objects_node src/pick_objects.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

#add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

