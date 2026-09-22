import os
from glob import glob

from setuptools import find_packages, setup

package_name = "ajr_path_planning"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.rviz"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Nagy Dávid",
    maintainer_email="nagy.david1@ga.sze.hu",
    description="A* és RRT alapú útvonaltervező és vizualizáció ROS 2 Humble alatt",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "map_server_node = ajr_path_planning.map_server_node:main",
            "astar_planner_node = ajr_path_planning.astar_planner_node:main",
            "rrt_planner_node = ajr_path_planning.rrt_planner_node:main",
            "dynamic_obstacle_node = ajr_path_planning.dynamic_obstacle_node:main",
            "evaluator_node = ajr_path_planning.evaluator_node:main",
        ],
    },
)