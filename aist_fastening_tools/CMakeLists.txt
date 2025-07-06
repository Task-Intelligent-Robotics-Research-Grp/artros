##########################################################################
# Set minimum required version of cmake, project name and compile options
##########################################################################
cmake_minimum_required(VERSION 3.5.0)
project(aist_fastening_tools)

## Compile as C++11, supported in ROS Kinetic and newer
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(dynamixel_workbench_controllers REQUIRED)
find_package(dynamixel_workbench_msgs REQUIRED)
find_package(ddynamic_reconfigure2 REQUIRED)
find_package(aist_utility REQUIRED)
find_package(aist_msgs REQUIRED)

##########################################################################
# Build
##########################################################################
include_directories(
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(screw_tool_controller
  src/screw_tool_controller.cpp
)

ament_target_dependencies(
  test_calibrator
    aist_utility
)

target_link_libraries(screw_tool_controller
  ${roscpp_LIBRARIES}
  ${catkin_LIBRARIES}
)

##########################################################################
# Install
##########################################################################
install(
  TARGETS  screw_tool_controller
  ARCHIVE  DESTINATION lib
  LIBRARY  DESTINATION lib
  RUNTIME  DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

########################
#  python executables  #
########################
ament_python_install_package(${PROJECT_NAME})

catkin_install_python(
  PROGRAMS
    scripts/screw_tool_test.py
    scripts/suction_tool_controller.py
    scripts/suction_tool_test.py
    scripts/conveni_suction_gripper_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
