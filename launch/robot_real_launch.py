from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="gears_nav2",
                # namespace="gears_nav2",
                executable="laser_ld_wall_detection_node",
            ),
            Node(
                package="gears_nav2",
                # namespace="gears_nav2",
                executable="zed_2i_custom_node",
            ),
            Node(
                package="gears_nav2",
                # namespace="gears_nav2",
                executable="zed_depth_subscriber_node",
            ),
        ]
    )
