cmake_minimum_required(VERSION 3.8)
project(aist_ftsensor)

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
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(ddynamic_reconfigure2 REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML-CPP yaml-cpp REQUIRED)

# catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    roscpp
    controller_interface
    geometry_msgs
    hardware_interface
    realtime_tools
  LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########
target_include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

add_library(
  dynpick_driver
    src/dynpick_driver.cpp
)

target_compile_definitions(
  dynpick_driver
    COMPOSITION_BUILDING_DLL
)

ament_target_dependencies(
  dynpick_driver
    rclcpp_components
    controller_manager
    hardware_interface
)

rclcpp_components_register_node(
  dynpick_driver
    PLUGIN     ${PROJECT_NAME}::dynpick_driver
    EXECUTABLE ${PROJECT_NAME}_node
)

add_library(
  ${PROJECT_NAME}_broadcaster SHARED
    src/aist_ftsensor_broadcaster.cpp
)

target_link_libraries(
  ${PROJECT_NAME}_broadcaster
    ${orocos_kdl_LIBRARIES}
    yaml-cpp
)

#############
## Install ##
#############
# Install targets
install(
  TARGETS dynpick_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Intall plugins
install(FILES aist_ftsensor_plugin.xml
              config/ftsensor_controller.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  PROGRAMS scripts/run_calibration.py
  DESTINATION lib/${PROJECT_NAME}
)
