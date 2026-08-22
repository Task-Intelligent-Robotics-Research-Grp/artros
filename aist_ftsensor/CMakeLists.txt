cmake_minimum_required(VERSION 3.20)
project(aist_ftsensor)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

##################
#  Dependencies  #
##################
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(PkgConfig)
pkg_check_modules(YAML-CPP REQUIRED yaml-cpp)

###########
#  Build  #
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

ament_auto_add_library(
  ${PROJECT_NAME}_broadcaster SHARED
    src/aist_ftsensor_broadcaster.cpp
)

#############
#  Install  #
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
install(
  FILES       aist_ftsensor_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS    scripts/run_calibration.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY   config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()
