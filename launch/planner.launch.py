import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share_directory = get_package_share_directory(
        "ajr_path_planning"
    )

    rviz_config_file = os.path.join(
        package_share_directory,
        "config",
        "planner.rviz",
    )

    return LaunchDescription([
        Node(
            package="ajr_path_planning",
            executable="map_server_node",
            name="map_server_node",
            output="screen",
        ),
        Node(
            package="ajr_path_planning",
            executable="astar_planner_node",
            name="astar_planner_node",
            output="screen",
        ),
        Node(
            package="ajr_path_planning",
            executable="rrt_planner_node",
            name="rrt_planner_node",
            output="screen",
        ),
        Node(
            package="ajr_path_planning",
            executable="dynamic_obstacle_node",
            name="dynamic_obstacle_node",
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_base_link",
            output="screen",
            arguments=[
                "--x",
                "0",
                "--y",
                "0",
                "--z",
                "0",
                "--roll",
                "0",
                "--pitch",
                "0",
                "--yaw",
                "0",
                "--frame-id",
                "map",
                "--child-frame-id",
                "base_link",
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=[
                "-d",
                rviz_config_file,
            ],
        ),
    ])