from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Launch twist_stamped_node
        Node(
            package='ackerman_ctrl_pkg',
            executable='twist_stamped_node',
            name='twist_stamped_node',
            output='screen'
        ),

        # Launch purple_suit_node
        # Node(
        #     package='ackerman_ctrl_pkg',
        #     executable='purple_suit_node',
        #     name='purple_suit_node',
        #     output='screen'
        # ),

        # Launch vicon_odom_node
        #Node(
        #    package='ackerman_ctrl_pkg',
        #    executable='vicon_odom_node',
        #    name='vicon_odom_node',
        #    output='screen'
        #),

        # # Launch twist_mux
        # Node(
        #     package='twist_mux',
        #     executable='twist_mux',
        #     name='twist_mux',
        #     output='screen'
        # ),

        #Launch vicon pose_publisher
        # Node(
        #     package='vicon',
        #     executable='pose_publisher',
        #     name='pose_publisher',
        #     output='screen'
        # ),

        # Launch static_transform_publisher
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                '--frame-id', 'base_link', '--child-frame-id', 'laser',
                '--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '3.1416'
            ],
            output='screen'
        ),
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                '--frame-id', 'base_footprint', '--child-frame-id', 'base_link',
                '--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0'
            ],
            output='screen'
        ),
    ])
