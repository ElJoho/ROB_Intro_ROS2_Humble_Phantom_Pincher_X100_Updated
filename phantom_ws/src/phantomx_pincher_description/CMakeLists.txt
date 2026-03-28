cmake_minimum_required(VERSION 3.5)
project(phantomx_pincher_description)

# Find dependencies
find_package(ament_cmake REQUIRED)

set(SCRIPTS_DIR scripts)
install(PROGRAMS
    ${SCRIPTS_DIR}/xacro2sdf_direct.bash
    ${SCRIPTS_DIR}/xacro2sdf.bash
    ${SCRIPTS_DIR}/xacro2urdf.bash
    DESTINATION lib/${PROJECT_NAME}
)

# Force reinstall when any xacro file changes
file(GLOB_RECURSE XACRO_FILES "${CMAKE_CURRENT_SOURCE_DIR}/urdf/*.xacro")
add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/xacro_stamp.txt
    COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_BINARY_DIR}/xacro_stamp.txt
    DEPENDS ${XACRO_FILES}
)
add_custom_target(check_xacro_files ALL DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/xacro_stamp.txt)

# Install directories
install(DIRECTORY config launch phantomx_pincher phantomx_pincher/meshes rviz urdf DESTINATION share/${PROJECT_NAME})

# 🔹 ADD THIS LINE to include all mesh subfolders (DAE, STL, etc.)
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

# Setup the project
ament_package()
