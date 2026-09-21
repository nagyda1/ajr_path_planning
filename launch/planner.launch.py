from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
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
    ])
