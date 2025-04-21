# ROS 2 Control Guide - Trabajo de Grado

Este repositorio contiene el material y los recursos desarrollados como parte de un trabajo de grado enfocado en la creación de un tutorial completo para el uso de `ros2_control`, una herramienta de ROS 2 para la gestión y control de hardware en robots.

El proyecto tiene como objetivo proporcionar una guía paso a paso para la implementación de plugins de hardware, configuración de controladores y simulación de robots utilizando `ros2_control`. Este tutorial está diseñado para ser accesible tanto para principiantes como para usuarios avanzados de ROS 2.

## Repositorios

Este proyecto está dividido en dos ramas principales:

1. **[main](https://github.com/XergonMaster/ros2_control_guide/tree/main)**: Contiene la versión estable del tutorial, con el código final y la documentación completa.
2. **[tutorial](https://github.com/XergonMaster/ros2_control_guide/tree/tutorial)**: Contiene el desarrollo paso a paso del tutorial, con ejemplos y explicaciones detalladas para cada sección.

## Estructura del Repositorio

El repositorio está organizado de la siguiente manera:

- **/docs**: Documentación adicional, diagramas y recursos relacionados con el trabajo de grado.
- **/src**: Código fuente de los ejemplos y plugins de hardware desarrollados.
- **/launch**: Archivos de lanzamiento para ejecutar los ejemplos.
- **/config**: Archivos de configuración para controladores y parámetros de `ros2_control`.
- **/description**: Archivos URDF/Xacro para la descripción del robot.
- **/rviz**: Configuraciones de RViz para visualizar los ejemplos.

## Requisitos

Para utilizar este tutorial, necesitarás tener instalado lo siguiente:

- **ROS 2** (versión recomendada: Humble Hawksbill).
- **ros2_control** y **ros2_controllers**.
- **rviz2** para la visualización.
- **xacro** para la descripción del robot.

Puedes instalar las dependencias necesarias con el siguiente comando:

```bash
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-xacro
