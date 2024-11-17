import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

# Define constants for better readability
PACKAGE_NAME = "gripper_ctrl_pkg"
ROBOT_DESCRIPTION_PARAM = "robot_description"
RVIZ_CONFIG_SUBDIR = "gripper/rviz"
URDF_SUBDIR = "urdf"
RVIZ_FILE = "gripper_view.rviz"
URDF_FILE = "gripper.urdf.xacro"
GAZEBO_LAUNCH_FILE = "gazebo.launch.py"

def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument("gui", default_value="true", description="Start RViz2 automatically with this launch file."),
        DeclareLaunchArgument("spawn_gazebo", default_value="true", description="Whether to spawn the robot in Gazebo."),
    ]

    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    spawn_gazebo = LaunchConfiguration("spawn_gazebo")

    # Get URDF via xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare(PACKAGE_NAME), URDF_SUBDIR, URDF_FILE]),
    ])
    robot_description = {ROBOT_DESCRIPTION_PARAM: robot_description_content}

    rviz_config_file = PathJoinSubstitution([FindPackageShare("bots_description"),RVIZ_CONFIG_SUBDIR, RVIZ_FILE])

    # Nodes
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # Load joint_state_broadcaster (for publishing joint states)
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    # Load gripper_controller (controller for your gripper, adjust as needed)
    load_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'gripper_controller'],
        output='screen'
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare("gazebo_ros").find("gazebo_ros"), "launch", GAZEBO_LAUNCH_FILE)
        ),
        condition=IfCondition(spawn_gazebo),
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'gripper'],
                        output='screen')

    # Controller manager node
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, PathJoinSubstitution([FindPackageShare(PACKAGE_NAME), 'config', 'controller_manager.yaml'])],  # Incluye el YAML de configuración
        output='screen',
        remappings=[
            ("~/robot_description", "/robot_description")],
    )


    # Delay loading controllers until after Gazebo spawns the entity
    delayed_gripper_controller_load = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[load_joint_state_broadcaster, load_gripper_controller]
        )
    )

    nodes = [
        rviz_node,
        robot_state_pub_node,
        gazebo,
        spawn_entity,
        # controller_manager_node,
        delayed_gripper_controller_load,  # Load controllers after robot spawn
    ]

    return LaunchDescription(declared_arguments + nodes)
