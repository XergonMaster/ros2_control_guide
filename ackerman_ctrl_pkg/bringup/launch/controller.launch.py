from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Obtén las rutas de los archivos de lanzamiento a incluir
    ackerman_ctrl_pkg_share = get_package_share_directory('ackerman_ctrl_pkg')
    spawn_ackerman_launch = os.path.join(ackerman_ctrl_pkg_share, 'launch', 'spawn_ackerman.launch.py')
    view_robot_launch = os.path.join(ackerman_ctrl_pkg_share, 'launch', 'view_robot.launch.py')

    return LaunchDescription([
        # Incluir el archivo de lanzamiento para spawnear el robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(spawn_ackerman_launch),
        ),

        # Incluir el archivo de lanzamiento para visualizar el robot en Rviz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(view_robot_launch),
            launch_arguments={'gui': 'false'}.items(),  # Pasar el argumento para desactivar la GUI
        ),

        # Nodo para spawnear el controlador
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['ackermann_steering_controller', '--controller-manager', '/controller_manager'],
            output='screen',
        ),

        # Nodo para spawnear el joint state broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen',
        ),
    ])
    