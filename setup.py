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
data_files.append(("share/" + package_name + "/launch", ["launch/robot_launch.py"]))


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="minegearscsu",
    maintainer_email="minegearscsu@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["gears_nav2_node = gears_nav2.gears_nav2_node:main"],
    },
)
