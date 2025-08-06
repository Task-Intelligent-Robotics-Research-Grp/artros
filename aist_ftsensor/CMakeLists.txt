cmake_minimum_required(VERSION 3.8)
project(aist_ftsensor)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(backward_ros REQUIRED)

set(
  THIS_PACKAGE_INCLUDE_DEPENDS
    rclcpp
    rclcpp_lifecycle
    rclpy
    controller_interface
    hardware_interface
    pluginlib
    realtime_tools
    std_srvs
    std_msgs
    geometry_msgs
    sensor_msgs
    ddynamic_reconfigure2
    aist_utility
    kdl_parser
    orocos_kdl
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML-CPP yaml-cpp REQUIRED)

###########
## Build ##
###########
# add_library(
#   dynpick_driver SHARED
#     src/dynpick_driver.cpp
# )

# target_compile_features(
#   dynpick_driver PUBLIC
#     cxx_std_17
# )

# target_link_libraries(
#   dynpick_driver PUBLIC
#     controller_interface::controller_interface
#     hardware_interface::hardware_interface
#     pluginlib::pluginlib
#     rclcpp::rclcpp
#     rclcpp_lifecycle::rclcpp_lifecycle
#     realtime_tools::realtime_tools
#     ${std_msgs_TARGETS}
#     ${geometry_msgs_TARGETS}
#     ${sensor_msgs_TARGETS}
# )

# pluginlib_export_plugin_description_file(
#   controller_interface
#     force_torque_sensor_broadcaster.xml
# )

add_library(
  ${PROJECT_NAME}_broadcaster SHARED
    src/aist_ftsensor_broadcaster.cpp
)

target_compile_features(
  ${PROJECT_NAME}_broadcaster PUBLIC
    cxx_std_17
)

target_link_libraries(
  ${PROJECT_NAME}_broadcaster PUBLIC
    controller_interface::controller_interface
    hardware_interface::hardware_interface
    pluginlib::pluginlib
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    realtime_tools::realtime_tools
    ${ddynamic_reconfigure2_TARGETS}
    ${aist_utility_TARGETS}
    ${std_srvs_TARGETS}
    ${std_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${kdl_parser_TARGETS}
    ${orocos_kdl_TARGETS}
    yaml-cpp
)

#############
## Install ##
#############
# Install targets
install(
#  TARGETS dynpick_driver ${PROJECT_NAME}_broadcaster
  TARGETS ${PROJECT_NAME}_broadcaster
  EXPORT  export_${PROJECT_NAME}_broadcaster
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

ament_export_targets(export_${PROJECT_NAME}_broadcaster HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
