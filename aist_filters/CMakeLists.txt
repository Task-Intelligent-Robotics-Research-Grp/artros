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
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

##############################################################################
# Build
##############################################################################
# Plugins
ament_auto_add_library(
  butterworth SHARED
    src/butterworth.cpp
)

# target_compile_features(
#   butterworth PUBLIC cxx_std_17
# )


# target_link_libraries(
#   butterworth PUBLIC
#     filters
#     pluginlib
# )

pluginlib_export_plugin_description_file(filters plugins_description.xml)

##############################################################################
# Install
##############################################################################
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS butterworth
  EXPORT  export_aist_filters
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export old-style CMake variables for includes
ament_export_include_directories("include/${PROJECT_NAME}")

ament_export_targets(export_aist_filters HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Boost
  pluginlib
  rclcpp
  filters
  aist_utility
)

ament_package()
