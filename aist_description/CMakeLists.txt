cmake_minimum_required(VERSION 3.20)
project(aist_description)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

#############
## Install ##
#############
install(
  PROGRAMS
    scripts/convert_stl_to_ply.py
    scripts/generate_urdf_from_meshes.py
  DESTINATION
    lib/${PROJECT_NAME}
)

##############################
#  environment hooks for gz  #
##############################
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in"
)
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in"
)

ament_auto_package(INSTALL_TO_SHARE launch devices objects parts scenes wrc)
