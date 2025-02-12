
markdown
Copy

# Tutorial: Creación de un Plugin de Hardware para ROS 2

En este tutorial, aprenderás a crear un plugin de hardware para ROS 2 utilizando `hardware_interface` y `pluginlib`. El plugin se integrará con `ros2_control` para controlar un robot simulado.

## 1. Creación del Paquete

Primero, crea un paquete ROS 2 para el plugin de hardware. Utiliza el siguiente comando:

```bash
ros2 pkg create --build-type ament_cmake arduino_interface --dependencies pluginlib --library-name arduino_test_HW

Este comando crea un paquete llamado arduino_interface con una biblioteca llamada arduino_test_HW.
2. Modificación del Archivo CMakeLists.txt

A continuación, modifica el archivo CMakeLists.txt para incluir las dependencias necesarias:
cmake
Copy

find_package(hardware_interface REQUIRED)

ament_target_dependencies(
  arduino_test_HW
  "pluginlib"
  hardware_interface
  rclcpp
  rclcpp_lifecycle
)

ament_export_dependencies(rclcpp hardware_interface pluginlib rclcpp_lifecycle)

Además, agrega la siguiente línea antes de ament_package() para exportar la descripción del plugin:
cmake
Copy

pluginlib_export_plugin_description_file(hardware_interface custom_hardware.xml)

3. Creación del Archivo custom_hardware.xml

Crea un archivo custom_hardware.xml para configurar la exportación del plugin:
xml
Copy

<library path="arduino_test_HW">
  <class name="arduino_interface/ArduinoTestHw" type="arduino_interface::ArduinoTestHw" base_class_type="hardware_interface::SystemInterface">
    <description>Arduino Test Hardware Interface</description>
  </class>
</library>

Run HTML
4. Implementación del Plugin
4.1. Cabecera del Plugin

En la cabecera del plugin, importa las siguientes librerías:
cpp
Copy

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

Define la clase ArduinoTestHw que hereda de hardware_interface::SystemInterface:
cpp
Copy

class ArduinoTestHw : public hardware_interface::SystemInterface
{
public:
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
    std::vector<double> hw_commands_;
    std::vector<double> last_hw_commands_;
    std::vector<double> hw_states_position_;
    std::vector<double> hw_states_velocity_;
};

4.2. Implementación de los Métodos

Implementa los métodos principales del plugin:
cpp
Copy

#include "hardware_interface/types/hardware_interface_type_values.hpp"

hardware_interface::CallbackReturn ArduinoTestHw::on_init(const hardware_interface::HardwareInfo &info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }
    hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    last_hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_states_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
        if (joint.command_interfaces.size() != 1)
        {
            RCLCPP_FATAL(rclcpp::get_logger("ArduinoTestHw"), "Joint '%s' has %ld command interfaces found. 1 expected.", joint.name.c_str(), joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.state_interfaces.size() != 2)
        {
            RCLCPP_FATAL(rclcpp::get_logger("ArduinoTestHw"), "Joint '%s' has %ld state interfaces found. 2 expected.", joint.name.c_str(), joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArduinoTestHw::on_configure(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
        hw_commands_[i] = 0.0;
        last_hw_commands_[i] = 0.0;
        hw_states_position_[i] = 0.0;
        hw_states_velocity_[i] = 0.0;
    }
    RCLCPP_INFO(rclcpp::get_logger("ArduinoTestHw"), "ArduinoTestHw Succesfully configured!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArduinoTestHw::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ArduinoTestHw::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_commands_[0]));
    return command_interfaces;
}

hardware_interface::CallbackReturn ArduinoTestHw::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ArduinoTestHw::read(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    (void)time;
    (void)period;
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArduinoTestHw::write(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    (void)time;
    (void)period;
    return hardware_interface::return_type::OK;
}

4.3. Exportación del Plugin

Finalmente, exporta la clase como un plugin de hardware_interface:
cpp
Copy

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(arduino_interface::ArduinoTestHw, hardware_interface::SystemInterface)

5. Creación de un Paquete de Prueba

Para probar el plugin, crea un nuevo paquete llamado real:
bash
Copy

ros2 pkg create --build-type ament_cmake real

Dentro de este paquete, crea las carpetas description, config, launch, y rviz. Luego, modifica el archivo CMakeLists.txt para instalar estas carpetas:
cmake
Copy

install(
  DIRECTORY 
    launch
    description
    rviz
    config
  DESTINATION share/${PROJECT_NAME}
)

5.1. Archivo robot.xacro

En la carpeta description, crea un archivo robot.xacro con la siguiente definición:
xml
Copy

<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">
    <xacro:property name="robot_namespace" value="robot"/>
    <xacro:property name="robot_description" value="$(arg robot_description)"/>

    <material name="black">
        <color rgba="0 0 0 1"/>
    </material>
    <material name="blue">
        <color rgba="0 0 1 1"/>
    </material>

    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.5 0.5 0.1"/>
            </geometry>
            <material name="blue"/>
        </visual>
    </link>

    <link name="link1">
        <visual>
            <geometry>
                <cylinder length="1.0" radius="0.1"/>
            </geometry>
            <material name="black"/>
        </visual>
    </link>

    <joint name="joint1" type="revolute">
        <parent link="base_link"/>
        <child link="link1"/>
        <origin xyz="0 0 0.55" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
        <limit effort="100" velocity="1.0" lower="-1.57" upper="1.57"/>
    </joint>

    <ros2_control name="motor_arduino" type="system">
        <hardware>
            <plugin>arduino_interface/ArduinoTestHw</plugin>
        </hardware>
        <joint name="joint1">
            <command_interface name="position"/>
            <state_interface name="position"/>
            <state_interface name="velocity"/>
        </joint>
    </ros2_control>
</robot>

Run HTML
5.2. Archivo de Lanzamiento

En la carpeta launch, crea un archivo de lanzamiento:
python
Copy

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('real')
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([pkg_share, 'description', 'robot.xacro']),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = os.path.join(pkg_share, 'rviz', 'robot.rviz')
    control_cfg_file = os.path.join(pkg_share, 'config', 'test.yaml')

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[control_cfg_file],
        remappings=[('~/robot_description', '/robot_description')],
        output='screen'
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        control_node,
        spawn_joint_state_broadcaster,
    ])

5.3. Archivo de Configuración

En la carpeta config, crea un archivo test.yaml:
yaml
Copy

controller_manager:
  ros__parameters:
    update_rate: 50  # Hz
    use_sim_time: false

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

6. Ejecución del Sistema

Para ejecutar el sistema, utiliza el siguiente comando:
bash
Copy

ros2 launch real launch_file.py

Esto lanzará el sistema con el plugin de hardware y el robot simulado.
