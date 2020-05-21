cmake_minimum_required(VERSION 3.0.2)
project(pick_objects)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  roscpp
  add_markers
)

catkin_package(
   LIBRARIES pick_objects_node
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pick_objects_node src/pick_objects.cpp)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

