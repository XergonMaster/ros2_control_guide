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
GAZEBO_LAUNCH_FILE = "gazebo.launch.py"

def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument("spawn_gazebo", default_value="true", description="Whether to spawn the robot in Gazebo."),
    ]

    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    spawn_gazebo = LaunchConfiguration("spawn_gazebo")


    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare("gazebo_ros").find("gazebo_ros"), "launch", GAZEBO_LAUNCH_FILE)
        ),
        condition=IfCondition(spawn_gazebo),
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'ackerman'],
                        output='screen',
)

    nodes = [
        gazebo,
        spawn_entity,
    ]

    return LaunchDescription(declared_arguments + nodes)
