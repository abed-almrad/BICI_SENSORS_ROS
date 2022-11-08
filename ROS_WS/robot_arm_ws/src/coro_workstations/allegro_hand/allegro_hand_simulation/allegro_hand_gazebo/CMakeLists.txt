cmake_minimum_required(VERSION 3.1.3)
project(allegro_hand_gazebo)

find_package(catkin REQUIRED)

catkin_package()

install(
  DIRECTORY controller
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
