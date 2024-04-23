from setuptools import find_packages, setup

package_name = "gears_nav2"
data_files = []

data_files.append(
    ("share/ament_index/resource_index/packages", ["resource/" + package_name])
)
data_files.append(("share/" + package_name, ["package.xml"]))

data_files.append(
    (
        "share/" + package_name + "/worlds/GearsNav2World/worlds",
        ["worlds/GearsNav2World/worlds/GearsNav2World.wbt"],
    )
)

data_files.append(
    (
        "share/" + package_name + "/worlds/GearsNav2World/worlds",
        ["worlds/GearsNav2World/worlds/3d_world.wbt"],
    )
)

data_files.append(
    ("share/" + package_name + "/resource", ["resource/gears_rover.urdf"])
)
data_files.append(("share/" + package_name + "/launch", ["launch/robot_launch.py"]))
data_files.append(
    ("share/" + package_name + "/launch", ["launch/robot_real_launch.py"])
)

data_files.append(
    (
        "share/" + package_name + "/worlds/GearsNav2World/protos",
        ["worlds/GearsNav2World/protos/GearsRover.proto"],
    )
)
data_files.append(
    ("share/" + package_name + "/controllers", ["controllers/gears_controller.py"])
)
setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="minegearscsu",
    maintainer_email="minegearscsu@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gears_nav2_node = gears_nav2.gears_nav2_node:main",
            "gears_3d_nav_node = gears_nav2.gears_3d_nav_node:main",
            "laser_wall_detection_node = gears_nav2.laser_wall_detection_node:main",
            "laser_ld_wall_detection_node = gears_nav2.laser_ld_wall_detection_node:main",
            "zed_2i_custom_node = gears_nav2.zed_2i_custom_node:main",
            "zed_depth_subscriber_node = gears_nav2.zed_depth_subscriber_node:main",
        ],
    },
)
