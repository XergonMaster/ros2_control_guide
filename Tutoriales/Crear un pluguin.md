# Tutorial: Creación de un Plugin de Hardware para ROS 2
En este tutorial, aprenderás a crear un plugin de hardware para ROS2 utilizando hardware_interface y `pluginlib`. El plugin se integrará con `ros2_control`.


## 1. Creación del Paquete
Primero, crea un paquete ROS 2 para el plugin de hardware. Utiliza el siguiente comando:

``` bash
ros2 pkg create --build-type ament_cmake [package_name] --dependencies  pluginlib --library-name [plugin_name]
```

Para este tutorial se creara el paquete llamado `arduino_interface` con la librería `arduino_test_HW` donde como guía pueden tomar cualquier nombre.

> [!TIP]
> `_HW` es una convención para la librería indicando que esta librería se va a utilizar como Hardware.

```bash
ros2 pkg create --build-type ament_cmake arduino_interface --dependencies pluginlib --library-name arduino_test_HW
```

## 2. Archivo de exportación del plugin
Crea un archivo `custom_hardware.xml` en la raíz del paquete para configurar la exportación del plugin:

```xml
<library path="[executable_name]">
    <class 
        name="[name_space]/[plugin_name]"
        type="[name_space]::[plugin_name]"
        base_class_type="hardware_interface::SystemInterface">
        <description>
            Whatever the user may to know
        </description>
    </class>
</library>
```
- `path`: Nombre/ubicación que recibe el ejecutable de la libreria en el `Cmakelist.txt`. 
- `name`: Nombre por el cual es llamado el plugin en la descripción del robot, usualmente igual que `type`.
- `type`: `name_space::plugin_name` del plugin


>[!Note]
>Al haber creado el paquete mediante el argumento `--library-name` el `executable_name` debe ser llamado de la misma forma.

Ejemplo aplicado:

```xml
<library path="arduino_test_HW">
    <class 
        name="arduino_interface/ArduinoTestHw"
        type="arduino_interface::ArduinoTestHw"
        base_class_type="hardware_interface::SystemInterface">
        <description>
            Hardware dedicado a hacer una prueba de comunicación con un arduino
        </description>
    </class>
</library>
```

## 3. Modificación del Archivo CMakeLists.txt

Para que el pluguin pueda ser correctamente exportado es necesario realizar ciertas modificaciones.


- Agregar la dependencia de `hardware_interface`

    ```cmake
    find_package(hardware_interface)
    ```
- Añade la dependencia `hardware_interface` dentro de `ament_target_dependencies([executable_name]...`

-  Incluye la siguiente linea para traer la configuración del arhivo `.xml`creado [anteriormente](## 2. Archivo de exportación del plugin
).

    ```cmake
    pluginlib_export_plugin_description_file(hardware_interface [export_file].xml)
    ```
    en este caso en particular:

    ```cmake
    pluginlib_export_plugin_description_file(hardware_interface custom_hardware.xml)
    ```

## 4. Implementación del Plugin como `hardware_component`

### 4.1. Cabecera del Plugin (`.hpp`)

En la cabecera del plugin `package_name/include/plugin_name/plugin_name.hpp`, importa las siguientes librerías:

```cpp
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
```
Define que la clase `PluguinName` herede de `hardware_interface::SystemInterface:`. Acontinuacíon se muestra la clase heredada, métodos de `ROS2_CONTROL` y unas variables utiles 
```cpp
namespace arduino_interface
{

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
}
```
> [!WARNING]
> Todos los metodos son necesarios.
- `hw_commands_` vector que contendra los comandos provenientes del controlador
- `hw_states_*`contenedor de velocidades/posiciones que necesita el controlador

> [!NOTE]
> Revisar cada controlador para saber cual es la itneracción, rangos, ciclo y retroalimentación

### 4.2. Implementación de los Métodos

Implementa los métodos principales del plugin en el cuerpo del plugin (`package_name/src/plugin_name.cpp`):

```cpp

#include "arduino_interface/arduino_test_HW.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace arduino_interface
{

    hardware_interface::CallbackReturn ArduinoTestHw::on_init(const hardware_interface::HardwareInfo &info)
    {
        RCLCPP_INFO(rclcpp::get_logger("ArduinoTestHw"), "change On init State");

        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }
        hw_commnds_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        last_hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_states_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        // Validate joints
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
        RCLCPP_INFO(rclcpp::get_logger("ArduinoTestHw"), "change on_configure State");

        (void)previous_state; // Para evitar la advertencia de parámetro no utilizado
        // Mockup implementation
        for (uint i = 0; i < info_.joints.size(); i++)
        {
            hw_commnds_[i] = 0.0;
            hw_commnds_[i] = 0.0;
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
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_commnds_[0]));
        // command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_commnds_[0]));
        // command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &hw_commnds_[1]));

        return command_interfaces;
    }

    hardware_interface::CallbackReturn ArduinoTestHw::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("ArduinoTestHw"), "change on_activate State");

        (void)previous_state; // Para evitar la advertencia de parámetro no utilizado
        // Mockup implementation
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ArduinoTestHw::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("ArduinoTestHw"), "change on_deactivate State");

        (void)previous_state; // Para evitar la advertencia de parámetro no utilizado
        // Mockup implementation
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type ArduinoTestHw::read(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)time;   // Para evitar la advertencia de parámetro no utilizado
        (void)period; // Para evitar la advertencia de parámetro no utilizado
        // Mockup implementation
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type ArduinoTestHw::write(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)time;   // Para evitar la advertencia de parámetro no utilizado
        (void)period; // Para evitar la advertencia de parámetro no utilizado
        // Mockup implementation
        return hardware_interface::return_type::OK;
    }

} 

```
### 4.3. Exportación del Plugin

Finalmente, exporta la clase como un plugin de `hardware_interface`:
```cpp

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(name_space::plugin_name hardware_interface::SystemInterface)
```
ejemplo aplicado:
```cpp
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(arduino_interface::ArduinoTestHw, hardware_interface::SystemInterface)
```
## 5. Prueba

Para probar el plugin, crea un nuevo paquete:
```bash
ros2 pkg create --build-type ament_cmake real
```
Dentro de este paquete, crea las carpetas description, config, launch, y rviz. Luego, modifica el archivo `CMakeLists.txt` para instalar estas carpetas:

```cmake
install(
  DIRECTORY 
    launch
    description
    rviz
    config
  DESTINATION share/${PROJECT_NAME}
)
```
### 5.1. Descripción del ronot

En la carpeta description, crea un archivo `robot.xacro` con la siguiente definición:
```xml
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
</robot>
```
Para añadir el pluguin que acabamos de crear se utiliza la etiqueta `ros2_control` como se muestra en el siguiente ejemplo, y se añade a `robot.xacro` dentro de etiqueta de `<robot>`

```xml  
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
```
### 5.2. Archivo de Lanzamiento

En la carpeta `launch`, crea un archivo de lanzamiento:
```python
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
```
### 5.3. Archivo de Configuración

En la carpeta `config`, crea un archivo `test.yaml`:
```yaml
controller_manager:
  ros__parameters:
    update_rate: 50  # Hz
    use_sim_time: false

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
```
### 6. Ejecución del Sistema

Tras compilar y verificar que todo se encuentre en orden, para ejecutar el sistema, utiliza el siguiente comando:
```bash
ros2 launch real launch_file.py
```
Esto lanzará el sistema con el plugin de hardware y el robot simulado.
