##########################################################################
# Set minimum required version of cmake, project name and compile options
##########################################################################
cmake_minimum_required(VERSION 3.5.0)
project(aist_precision_gripper)

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
find_package(rclcpp_action REQUIRED)
find_package(dynamixel_workbench_msgs REQUIRED)
find_package(ddynamic_reconfigure2 REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

###########
#  Build  #
###########
add_library(
  ${PROJECT_NAME} SHARED
    src/precision_gripper_controller.cpp
)

target_include_directories(
  ${PROJECT_NAME} PRIVATE
    $<INSTALL_INTERFACE:include>
)

target_compile_definitions(
  ${PROJECT_NAME} PRIVATE
    COMPOSITION_BUILDING_DLL
)

ament_target_dependencies(
  ${PROJECT_NAME}
    rclcpp_components
    rclcpp_action
    dynamixel_workbench_msgs
    ddynamic_reconfigure2
    control_msgs
    sensor_msgs
)

rclcpp_components_register_nodes(
  ${PROJECT_NAME}
    PLUGIN     ${PROJECT_NAME}::PrecisionGripperController
    EXECUTABLE precision_gripper_controller_node
)

#############
#  Install  #
#############
install(
  TARGETS  ${PROJECT_NAME}
  EXPORT   export_${PROJECT_NAME}
  ARCHIVE  DESTINATION lib
  LIBRARY  DESTINATION lib
  RUNTIME  DESTINATION bin
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

########################
#  Python executables  #
########################
ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    scripts/test_gripper_client.py
  DESTINATION
    lib/${PROJECT_NAME}
)

#############
#  Exports  #
#############
ament_export_libraries(${PROJECT_NAME})

ament_package()
