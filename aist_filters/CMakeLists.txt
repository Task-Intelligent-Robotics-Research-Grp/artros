cmake_minimum_required(VERSION 3.14.4)
project(aist_filters)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#######################
#  Find dependencies  #
#######################
# find_package(ament_cmake_auto REQUIRED)
# ament_auto_find_build_dependencies()
find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(filters REQUIRED)
find_package(aist_utility REQUIRED)

###########
#  Build  #
###########
include_directories(
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

generate_parameter_library(
  butterworth_filter_parameters
    src/butterworth_filter_parameters.yaml
)

add_library(
  butterworth_filter SHARED
    src/butterworth_filter.cpp)

target_include_directories(
  butterworth_filter INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(
  butterworth_filter PUBLIC
    rclcpp::rclcpp
    butterworth_filter_parameters
    ${geometry_msgs_TARGETS}
    ${filters_TARGETS}
    ${aist_utility_TARGETS}
)

##########
#  Test  #
##########
# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)

#   ament_add_gtest(
#     butterworth_test
#       test/test_butterworth.cpp
#   )

#   target_link_libraries(
#     butterworth_test
#       butterworth
#   )
# endif()

#############
#  Install  #
#############
pluginlib_export_plugin_description_file(filters ${PROJECT_NAME}.xml)

install(
  DIRECTORY   include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS butterworth_filter butterworth_filter_parameters
  EXPORT  export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  filters
  pluginlib
  rclcpp
  aist_utility
)

ament_package()
# ament_auto_package(
#   USE_SCOPED_HEADER_INSTALL_DIR
# )
