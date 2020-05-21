cmake_minimum_required(VERSION 3.0.2)
project(add_markers)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_runtime
  roscpp
  std_msgs
  message_generation
  visualization_msgs
)

add_service_files(
  DIRECTORY
  srv
  FILES
  ManipMarker.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES add_markers_node
  CATKIN_DEPENDS roscpp std_msgs 
  DEPENDS system_lib
)

catkin_package(CATKIN_DEPENDS std_msgs
  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(${PROJECT_NAME}_node src/add_markers.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

add_dependencies(add_markers_node add_markers_gencpp)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

