from setuptools import find_packages, setup

package_name = "ajr_path_planning"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/planner.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="nagyda1",
    maintainer_email="nagyda1@example.com",
    description="A* es RRT alapu utvonaltervezo es vizualizacio ROS 2 Humble alatt - AJR nagy feleves",
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
