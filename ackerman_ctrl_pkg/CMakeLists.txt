cmake_minimum_required(VERSION 3.8)
project(ackerman_ctrl_pkg)

# Añade opciones de compilación para mayor información de depuración
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Encuentra dependencias necesarias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# Añadir ejecutable
add_executable(twist_stamped_node
  src/twist_stamped_node.cpp
)
add_executable(purple_suit_node
  src/purple_suit.cpp
)
add_executable(vicon_odom_node
  src/vicon_odom.cpp
)
# Vincular bibliotecasament_target_dependencies(purple_suit

ament_target_dependencies(twist_stamped_node
  rclcpp
  geometry_msgs
  std_msgs
)

ament_target_dependencies(purple_suit_node
  rclcpp
  geometry_msgs
  std_msgs
)

ament_target_dependencies(vicon_odom_node
  rclcpp
  geometry_msgs
  std_msgs
  nav_msgs
  tf2_ros
)
# Instalar el ejecutable

install(TARGETS
  twist_stamped_node
  purple_suit_node
  vicon_odom_node
  DESTINATION lib/${PROJECT_NAME}
)


# Añade los directorios de inclusión
include_directories(
  include
)

# Añade la instalación de archivos de configuración y lanzamiento
install(
  DIRECTORY 
    description/launch 
    description/urdf 
    description/hardware_interface 
    bringup/launch 
    bringup/config
  DESTINATION share/${PROJECT_NAME}
)


# Añade los nodos y otros ejecutables si los tienes
# Por ejemplo, si tienes nodos en tu paquete, agrégalos de la siguiente manera:
# add_executable(my_node src/my_node.cpp)
# ament_target_dependencies(my_node rclcpp ...)

# Si tu paquete define algún plugin, asegúrate de exportar el archivo de descripción de plugins
# pluginlib_export_plugin_description_file(package_name path_to_description_file)

# Exporta las dependencias para que otros paquetes puedan utilizarlas
ament_export_dependencies(
  rclcpp
  controller_manager
  hardware_interface
  pluginlib
  urdf
  xacro
  ros2_control
  ackermann_msgs
)

# Marca el paquete para su instalación
ament_package()
