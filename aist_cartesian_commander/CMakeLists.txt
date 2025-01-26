cmake_minimum_required(VERSION 3.5.0)
project(aist_cartesian_commander)

## Compile as C++17, supported in ROS melodic and newer
enable_language(CXX)

set(CMAKE_CXX_STANDARD		17)
set(CMAKE_CXX_STANDARD_REQUIRED	ON)
set(CMAKE_CXX_EXTENSIONS	OFF)
set(CMAKE_CXX_FLAGS		"-Wall -O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE	"")
set(CMAKE_CXX_FLAGS_DEBUG	"-g -DPROFILE")
set(CMAKE_CXX_BUILD_TYPE	Release)

find_package(
  catkin REQUIRED
  COMPONENTS
    roscpp
    geometry_msgs
    tf2_geometry_msgs
    genmsg
    message_generation
    actionlib_msgs
    actionlib
    pluginlib
    nodelet
)

catkin_python_setup()

add_action_files(
  FILES
    TrackWithContact.action
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    actionlib
    geometry_msgs
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  src
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    message_runtime
    tf2_geometry_msgs
    pluginlib
    nodelet
)

add_library(
  ${PROJECT_NAME}
  src/nodelet.cpp
  src/CartesianCommander.cpp
)

add_dependencies(
  ${PROJECT_NAME}
  ${${PREOJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
)

#############
## Install ##
#############
install(
  TARGETS
    ${PROJECT_NAME}
  ARCHIVE
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
