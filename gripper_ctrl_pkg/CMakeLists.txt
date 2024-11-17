cmake_minimum_required(VERSION 3.8)
project(gripper_ctrl_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  geometry_msgs
)

find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Generar la biblioteca de parámetros
generate_parameter_library(gripper_ctrl_params
  src/gripper_ctrl_params.yaml  # Archivo YAML que contiene tus parámetros
)

# Create the library for the controller
add_library(${PROJECT_NAME} 
  SHARED 
  src/gripper_ctrl.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)  # Use C++17

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link against necessary dependencies
ament_target_dependencies(
  ${PROJECT_NAME} PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "GRIPPER_CTRL_PKG_BUILDING_DLL")

# Link the parameter library
target_link_libraries(${PROJECT_NAME} PUBLIC gripper_ctrl_params)
# Export the plugin description
pluginlib_export_plugin_description_file(controller_interface gripper_ctrl_pkg_plugins.xml)

# Install targets
install(TARGETS 
  ${PROJECT_NAME} 
  gripper_ctrl_params  # Add the parameter library to the install target
  EXPORT export_gripper_ctrl_pkg
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ 
  DESTINATION include)

# Ament package
ament_export_targets(export_gripper_ctrl_pkg HAS_LIBRARY_TARGET)
ament_export_libraries(gripper_ctrl gripper_ctrl_params)  # Export both libraries
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
