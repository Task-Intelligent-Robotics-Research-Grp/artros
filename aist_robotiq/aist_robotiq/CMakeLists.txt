cmake_minimum_required(VERSION 3.8)
project(aist_robotiq)

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
find_package(rclcpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(common_interfaces REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(pluginlib REQUIRED)

#find_package(gz_sim_vendor REQUIRED)
find_package(ros_gz_sim REQUIRED)
#find_package(gz_plugin_vendor REQUIRED)
#find_package(ros-gz-plugin REQUIRED)

find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

include_directories(
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)


link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(
  ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} include
)

###############################
## gazebo_mimic_joint_plugin ##
###############################
add_library(
  gz_mimic_joint_plugin SHARED
    src/mimic_joint_plugin.cpp
)

##################
## ft300_driver ##
##################
add_library(
  ft300_driver SHARED
    src/ft300_driver.cpp
)

target_compile_definitions(
  ft300_driver PRIVATE
    COMPOSITION_BUILDING_DLL
)

ament_target_dependencies(
  ft300_driver
    rclcpp_components
    controller_manager
    controller_interface
)

rclcpp_components_register_node(
  ft300_driver
    PLUGIN "${PROJECT_NAME}::ft300_driver"
    EXECUTABLE ft300_driver_node
)

#############
## Install ##
#############
install(
  TARGETS
    gz_mimic_joint_plugin
  EXPORT   export_${PROJECT_NAME}
  LIBRARY  DESTINATION lib
  ARCHIVE  DESTINATION lib
  RUNTIME  DESTINATION bin
)

install(
  DIRECTORY   include
  DESTINATION include
)

install(
  DIRECTORY   config launch meshes urdf
  DESTINATION share/${PROJECT_NAME}/
)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    scripts/cmodel_contoller.py
    scripts/cmodel_rtu_driver.py
    scripts/cmodel_tcp_driver.py
    scripts/cmodel_urcap_driver.py
    scripts/epick_controller.py
    scripts/test_cmodel_client.py
    scripts/test_epick_client.py
  DESTINATION lib/${PROJECT_NAME}
)

############
##  test  ##
############
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
