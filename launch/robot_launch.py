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
import xacro


def generate_launch_description():
    package_name = "gears_nav2"
    package_dir = os.path.join(get_package_share_directory(package_name))

    use_sim_time = LaunchConfiguration("use_sim_time", default=False)

    webots = WebotsLauncher(
        world=os.path.join(package_dir, "worlds", "PIKABOT", "worlds", "pikabot.wbt")
    )

    my_robot_driver = Node(
        package="webots_ros2_driver",
        executable="driver",
        output="screen",
        additional_env={"WEBOTS_CONTROLLER_URL": controller_url_prefix() + "pikabot"},
        parameters=[
            {
                # "robot_description": robot_description,
                "use_sim_time": use_sim_time,
                "set_robot_state_publisher": False,
            },
        ],
    )
    return LaunchDescription(
        [
            webots,
            my_robot_driver,
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=webots,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                )
            ),
        ]
    )
