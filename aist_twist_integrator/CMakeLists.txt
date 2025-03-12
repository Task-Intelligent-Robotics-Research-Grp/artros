cmake_minimum_required(VERSION 3.5.0)
project(aist_twist_integrator)

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
    realtime_tools
    geometry_msgs
    tf2_geometry_msgs
    ddynamic_reconfigure
    genmsg
    actionlib_msgs
    actionlib
    pluginlib
    nodelet
)

catkin_python_setup()

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
    actionlib_msgs
    tf2_geometry_msgs
    pluginlib
    nodelet
)

add_library(
  ${PROJECT_NAME}
  src/nodelet.cpp
  src/TwistIntegrator.cpp
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

catkin_install_python(
  PROGRAMS
    scripts/test_twist_integrator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
