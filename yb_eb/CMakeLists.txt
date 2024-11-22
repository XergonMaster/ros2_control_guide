cmake_minimum_required(VERSION 3.8)
project(yb_eb)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
 src/yb_eb_HW.cpp
 src/eb_serial.cpp
 )

 # Construye el ejecutable 'beep_node'

add_executable(beep_node src/beep_node.cpp)
target_link_libraries(beep_node ${PROJECT_NAME})  # Usa la biblioteca principal
ament_target_dependencies(beep_node rclcpp std_msgs)

# Construye el ejecutable 'servo_node'
add_executable(servo_node src/servo_node.cpp)
target_link_libraries(servo_node ${PROJECT_NAME})  # Usa la biblioteca principal
ament_target_dependencies(servo_node rclcpp std_msgs)


# Instala los ejecutables
install(TARGETS 
  beep_node servo_node 
  DESTINATION lib/${PROJECT_NAME}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  pluginlib
)




# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
# target_compile_definitions(yb_eb_HW PRIVATE "YB_EB_BUILDING_LIBRARY")
# target_compile_definitions(${PROJECT_NAME} PRIVATE "JOINT_STATE_BROADCASTER_BUILDING_DLL")
# target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(hardware_interface custom_hardware.xml)

install(
  DIRECTORY include/
  DESTINATION include
)
# install(
#   TARGETS ${PROJECT_NAME}
#   DESTINATION lib
# )

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(FILES custom_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp hardware_interface pluginlib rclcpp_lifecycle)


ament_package()
