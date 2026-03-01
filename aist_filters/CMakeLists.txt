cmake_minimum_required(VERSION 3.14.4)
project(aist_filters)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

##############################################################################
# Find dependencies
##############################################################################
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Boost QUIET COMPONENTS headers)
if(NOT Boost_headers_FOUND)
  find_package(Boost REQUIRED)
endif()

include_directories(
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)

##############################################################################
# Build
##############################################################################
# Plugins
ament_auto_add_library(
  butterworth SHARED
    src/butterworth.cpp)

ament_target_dependencies(
  butterworth
    filters
    pluginlib
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

##############################################################################
# Install
##############################################################################

# Export old-style CMake variables for includes
ament_export_include_directories("include/${PROJECT_NAME}")

ament_export_dependencies(
  Boost
  pluginlib
  rclcpp
)

# Install pluginlib xml
ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
)
