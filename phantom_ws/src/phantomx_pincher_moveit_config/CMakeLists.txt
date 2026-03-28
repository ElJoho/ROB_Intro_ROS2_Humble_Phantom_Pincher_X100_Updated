cmake_minimum_required(VERSION 3.5)
project(phantomx_pincher_moveit_config)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(phantomx_pincher_description REQUIRED)
find_package(xacro REQUIRED)

# Generate SRDF from xacro at build time
xacro_add_xacro_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/srdf/phantomx_pincher.srdf.xacro
  OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/srdf/phantomx_pincher.srdf
  REMAP name:=phantomx_pincher use_real_gripper:=true
)
add_custom_target(srdf_generation ALL DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/srdf/phantomx_pincher.srdf)

# Install directories
install(DIRECTORY config launch rviz srdf DESTINATION share/${PROJECT_NAME})

# Setup the project
ament_package()
