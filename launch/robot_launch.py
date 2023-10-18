import os
import pathlib
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.utils import controller_url_prefix
from webots_ros2_driver.webots_controller import WebotsController
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    package_name = "gears_nav2"
    package_dir = os.path.join(get_package_share_directory(package_name))
    # robot_description_path = pathlib.Path(
    #     os.path.join(package_dir, "resource", "gears_rover.urdf")
    # ).read_text()
    robot_description_path = os.path.join(package_dir, "resource", "gears_rover.urdf")
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)

    webots = WebotsLauncher(
        world=os.path.join(
            package_dir, "worlds", "GearsNav2World", "worlds", "GearsNav2World.wbt"
        )
    )

    my_robot_driver = WebotsController(
        robot_name="Gears3",
        parameters=[
            {
                "robot_description": robot_description_path,
                # "set_robot_state_publisher": True,
            },
        ],
    )

    base_link_to_laser = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0.01", "0", "0", "0", "base_link", "LDS-01"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            webots,
            base_link_to_laser,
            my_robot_driver,
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=webots,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                )
            ),
        ]
    )
