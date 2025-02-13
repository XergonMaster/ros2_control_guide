from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
 
from launch.substitutions import Command, FindExecutable,PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

import os

def generate_launch_description():
    # Path to the robot description package
    pkg_share = get_package_share_directory('real')

    # Path to the robot description file
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [pkg_share, 'description', 'robot.xacro']
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Path to the RViz configuration file
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'robot.rviz')

    control_cfg_file = os.path.join(
        pkg_share,
        'config',
        'test.yaml'
    )
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[control_cfg_file],
        # don't forget to remap the robot_description
        remappings=[('~/robot_description', '/robot_description')],
        output='screen'
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    spawn_joint_state_broadcaster= Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen')
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
        # spawn_joint_state_broadcaster,
    ])