from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart


def generate_launch_description():
    # Ruta al archivo de configuración YAML
    config_file_path = os.path.join(
        get_package_share_directory('ackerman_ctrl_pkg'),
        'config',
        'real_robot.yaml'
    )

    # Incluir el launch del visualizador (opcional)
    view_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ackerman_ctrl_pkg"),
                "launch",
                "view_robot.launch.py",
            )
        ),
        launch_arguments={"description_file": "bot_real.urdf.xacro", "gui": "false"}.items(),
    )

    # Comando para cargar el archivo URDF/Xacro del robot
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ackerman_ctrl_pkg"), "urdf", "bot_real.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Nodo para ejecutar ros2_control_node
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[config_file_path],
        remappings=[('~/robot_description', '/robot_description')],
        output='screen'
    )

    # Nodo para el joint_state_broadcaster
    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # Nodo para el ackermann_steering_controller
    ackermann_steering_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ackermann_steering_controller'],
        output='screen'
    )

    # Evento para iniciar `joint_state_spawner` tras el inicio de `control_node`
    joint_state_event = RegisterEventHandler(
        OnProcessStart(
            target_action=control_node,
            on_start=[joint_state_spawner]
        )
    )

    # Evento para iniciar `ackermann_steering_controller_spawner` tras el inicio de `joint_state_spawner`
    steering_controller_event = RegisterEventHandler(
        OnProcessStart(
            target_action=joint_state_spawner,
            on_start=[ackermann_steering_controller_spawner]
        )
    )

    # Retorno de la descripción completa del lanzamiento
    return LaunchDescription([
        view_robot_launch,
        control_node,
        joint_state_event,
        steering_controller_event,
    ])
