cmake_minimum_required(VERSION 2.8.3)
project(allegro_hand_taxels)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  tf2_ros
  geometry_msgs
  std_msgs
  roscpp
  message_generation
  visualization_msgs
)

add_message_files(
  FILES
  coordinates_saving.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES allegro_hand_taxels
  CATKIN_DEPENDS   tf2_ros geometry_msgs visualization_msgs roscpp std_msgs message_runtime
#  DEPENDS
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)


## Declare executables
add_executable(coordinates_listener src/coordinates_listener.cpp)
add_executable(saving_node src/saving_node.cpp)
add_executable(sensor_emulator src/sensor_emulator.cpp)
target_link_libraries(coordinates_listener ${catkin_LIBRARIES})
target_link_libraries(saving_node ${catkin_LIBRARIES})
target_link_libraries(sensor_emulator ${catkin_LIBRARIES})
#############
## Install ##
#############

install(TARGETS
  coordinates_listener
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch files
##install(DIRECTORY launch
##  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
##  FILES_MATCHING PATTERN "*.launch"
##)
