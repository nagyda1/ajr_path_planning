import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray

from ajr_path_planning.metrics_logger import write_result
from ajr_path_planning.planning_utils import GridMap, TreeNode, rrt_search

START_WORLD = (0.75, 0.75)


class RrtPlannerNode(Node):
    def __init__(self):
        super().__init__("rrt_planner_node")

        self.grid = None

        self.create_subscription(
            OccupancyGrid,
            "/map",
            self.map_callback,
            10,
        )

        self.create_subscription(
            PoseStamped,
            "/goal_pose",
            self.goal_callback,
            10,
        )

        self.path_publisher = self.create_publisher(
            Path,
            "/rrt/path",
            10,
        )

        self.marker_publisher = self.create_publisher(
            MarkerArray,
            "/rrt/markers",
            10,
        )

        self.get_logger().info(
            "RrtPlannerNode elindult. Várakozás a /map és /goal_pose üzenetekre."
        )

    def map_callback(self, msg: OccupancyGrid):
        self.grid = GridMap(
            width=msg.info.width,
            height=msg.info.height,
            resolution=msg.info.resolution,
            origin=(
                msg.info.origin.position.x,
                msg.info.origin.position.y,
            ),
            data=list(msg.data),
        )

    def goal_callback(self, msg: PoseStamped):
        if self.grid is None:
            self.get_logger().warn("Még nem érkezett /map üzenet.")
            return

        goal_world = (
            msg.pose.position.x,
            msg.pose.position.y,
        )

        start_node = TreeNode(
            x=START_WORLD[0],
            y=START_WORLD[1],
        )

        goal_node = TreeNode(
            x=goal_world[0],
            y=goal_world[1],
        )

        self.get_logger().info(
            f"RRT keresés indítása: start={START_WORLD}, cél={goal_world}"
        )

        start_time = time.perf_counter()

        path_nodes, tree_nodes = rrt_search(
            self.grid,
            start_node,
            goal_node,
        )

        elapsed_time_ms = (time.perf_counter() - start_time) * 1000.0

        self.publish_tree(tree_nodes)

        if path_nodes is None:
            write_result(
                algorithm="RRT",
                start=START_WORLD,
                goal=goal_world,
                success=False,
                path_length_m=None,
                elapsed_time_ms=elapsed_time_ms,
                point_count=0,
                tree_node_count=len(tree_nodes),
            )

            self.get_logger().warn(
                f"Nem található érvényes RRT útvonal. "
                f"Futásidő: {elapsed_time_ms:.3f} ms, "
                f"fa csomópontjai: {len(tree_nodes)}."
            )
            return

        world_points = [
            (node.x, node.y)
            for node in path_nodes
        ]

        path_length = self.calculate_path_length(world_points)

        write_result(
            algorithm="RRT",
            start=START_WORLD,
            goal=goal_world,
            success=True,
            path_length_m=path_length,
            elapsed_time_ms=elapsed_time_ms,
            point_count=len(world_points),
            tree_node_count=len(tree_nodes),
        )

        self.publish_path(world_points)
        self.publish_path_markers(world_points)

        self.get_logger().info(
            f"RRT útvonal megtalálva: {len(world_points)} pont, "
            f"hossz: {path_length:.3f} m, "
            f"futásidő: {elapsed_time_ms:.3f} ms, "
            f"fa csomópontjai: {len(tree_nodes)}."
        )

    def calculate_path_length(self, world_points):
        if len(world_points) < 2:
            return 0.0

        path_length = 0.0

        for index in range(1, len(world_points)):
            path_length += math.dist(
                world_points[index - 1],
                world_points[index],
            )

        return path_length

    def publish_path(self, world_points):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"

        for wx, wy in world_points:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_publisher.publish(path_msg)

    def publish_tree(self, tree_nodes):
        marker_array = MarkerArray()

        tree_marker = Marker()
        tree_marker.header.frame_id = "map"
        tree_marker.header.stamp = self.get_clock().now().to_msg()
        tree_marker.ns = "rrt_tree"
        tree_marker.id = 0
        tree_marker.type = Marker.LINE_LIST
        tree_marker.action = Marker.ADD
        tree_marker.scale.x = 0.015
        tree_marker.color.r = 0.0
        tree_marker.color.g = 0.7
        tree_marker.color.b = 1.0
        tree_marker.color.a = 0.5

        for node in tree_nodes:
            if node.parent is not None:
                tree_marker.points.append(
                    Point(
                        x=node.parent.x,
                        y=node.parent.y,
                        z=0.0,
                    )
                )
                tree_marker.points.append(
                    Point(
                        x=node.x,
                        y=node.y,
                        z=0.0,
                    )
                )

        marker_array.markers.append(tree_marker)
        self.marker_publisher.publish(marker_array)

    def publish_path_markers(self, world_points):
        if not world_points:
            return

        marker_array = MarkerArray()
        timestamp = self.get_clock().now().to_msg()

        path_marker = Marker()
        path_marker.header.frame_id = "map"
        path_marker.header.stamp = timestamp
        path_marker.ns = "rrt_path"
        path_marker.id = 1
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.07
        path_marker.color.r = 1.0
        path_marker.color.g = 0.5
        path_marker.color.b = 0.0
        path_marker.color.a = 1.0
        path_marker.points = [
            Point(x=wx, y=wy, z=0.02)
            for wx, wy in world_points
        ]

        start_marker = Marker()
        start_marker.header.frame_id = "map"
        start_marker.header.stamp = timestamp
        start_marker.ns = "rrt_start"
        start_marker.id = 2
        start_marker.type = Marker.SPHERE
        start_marker.action = Marker.ADD
        start_marker.pose.position = Point(
            x=world_points[0][0],
            y=world_points[0][1],
            z=0.0,
        )
        start_marker.pose.orientation.w = 1.0
        start_marker.scale.x = 0.2
        start_marker.scale.y = 0.2
        start_marker.scale.z = 0.2
        start_marker.color.r = 0.0
        start_marker.color.g = 0.0
        start_marker.color.b = 1.0
        start_marker.color.a = 1.0

        goal_marker = Marker()
        goal_marker.header.frame_id = "map"
        goal_marker.header.stamp = timestamp
        goal_marker.ns = "rrt_goal"
        goal_marker.id = 3
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD
        goal_marker.pose.position = Point(
            x=world_points[-1][0],
            y=world_points[-1][1],
            z=0.0,
        )
        goal_marker.pose.orientation.w = 1.0
        goal_marker.scale.x = 0.2
        goal_marker.scale.y = 0.2
        goal_marker.scale.z = 0.2
        goal_marker.color.r = 1.0
        goal_marker.color.g = 0.0
        goal_marker.color.b = 0.0
        goal_marker.color.a = 1.0

        marker_array.markers.append(path_marker)
        marker_array.markers.append(start_marker)
        marker_array.markers.append(goal_marker)
        self.marker_publisher.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = RrtPlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()