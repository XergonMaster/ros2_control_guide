cmake_minimum_required(VERSION 3.8)
project(yb_board)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Encuentra dependencias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED)
find_package(std_msgs REQUIRED)

# Incluye los directorios de encabezado
include_directories(include ${Boost_INCLUDE_DIRS})

# Construye la biblioteca 'rosmaster' (control de comunicación serial)
add_library(rosmaster SHARED src/rosmaster.cpp)
ament_target_dependencies(rosmaster Boost)

# Construye el ejecutable 'beep_node' que usa la biblioteca
add_executable(beep_node src/beep_node.cpp src/rosmaster.cpp)
ament_target_dependencies(beep_node rclcpp std_msgs)  # Añadir las dependencias necesarias

add_executable(servo_node src/servo_node.cpp src/rosmaster.cpp)
ament_target_dependencies(servo_node rclcpp std_msgs)  # Añadir las dependencias necesarias

install(TARGETS 
beep_node servo_node 
DESTINATION lib/${PROJECT_NAME})


# Configuración para pruebas
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Opciones para omitir copyright y licencias
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


# Instala el paquete
install(TARGETS
  rosmaster
  LIBRARY DESTINATION lib
)

# Instala encabezados y otros archivos requeridos
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()
