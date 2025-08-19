cmake_minimum_required(VERSION 3.8)
project(aist_bringup)

find_package(ament_cmake_core REQUIRED)

#############
## Install ##
#############
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
