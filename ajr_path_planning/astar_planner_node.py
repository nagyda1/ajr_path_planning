import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray

from ajr_path_planning.planning_utils import GridMap, astar_search
from ajr_path_planning.metrics_logger import write_result

START_WORLD = (0.75, 0.75)


class AstarPlannerNode(Node):
    def __init__(self):
        super().__init__("astar_planner_node")

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
            "/astar/path",
            10,
        )

        self.marker_publisher = self.create_publisher(
            MarkerArray,
            "/astar/markers",
            10,
        )

        self.get_logger().info(
            "AstarPlannerNode elindult. Várakozás a /map és /goal_pose üzenetekre."
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

        start_grid = self.grid.world_to_grid(*START_WORLD)

        goal_grid = self.grid.world_to_grid(
            msg.pose.position.x,
            msg.pose.position.y,
        )

        self.get_logger().info(
            f"A* keresés indítása: start={start_grid}, cél={goal_grid}"
        )

        start_time = time.perf_counter()

        path_cells = astar_search(
            self.grid,
            start_grid,
            goal_grid,
        )

        elapsed_time_ms = (time.perf_counter() - start_time) * 1000.0

        if path_cells is None:
            write_result(
                algorithm="A*",
                start=START_WORLD,
                goal=(
                    msg.pose.position.x,
                    msg.pose.position.y,
                ),
                success=False,
                path_length_m=None,
                elapsed_time_ms=elapsed_time_ms,
                point_count=0,
                tree_node_count=0,
            )

            self.get_logger().warn(
                f"Nem található érvényes útvonal. Futásidő: {elapsed_time_ms:.3f} ms."
            )
            return

        world_points = [
            self.grid.grid_to_world(gx, gy)
            for gx, gy in path_cells
        ]

        path_length = self.calculate_path_length(world_points)
        write_result(
            algorithm="A*",
            start=START_WORLD,
            goal=(
                msg.pose.position.x,
                msg.pose.position.y,
            ),
            success=True,
            path_length_m=path_length,
            elapsed_time_ms=elapsed_time_ms,
            point_count=len(world_points),
            tree_node_count=0,
        )

        self.publish_path(world_points)
        self.publish_markers(world_points)

        self.get_logger().info(
            f"Útvonal megtalálva: {len(world_points)} pont, "
            f"hossz: {path_length:.3f} m, "
            f"futásidő: {elapsed_time_ms:.3f} ms."
        )

    def calculate_path_length(self, world_points):
        if len(world_points) < 2:
            return 0.0

        path_length = 0.0

        for index in range(1, len(world_points)):
            previous_point = world_points[index - 1]
            current_point = world_points[index]

            path_length += math.dist(
                previous_point,
                current_point,
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

    def publish_markers(self, world_points):
        marker_array = MarkerArray()

        line_marker = Marker()
        line_marker.header.frame_id = "map"
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = "astar_path"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.05
        line_marker.color.r = 0.0
        line_marker.color.g = 1.0
        line_marker.color.b = 0.0
        line_marker.color.a = 1.0
        line_marker.points = [
            Point(x=wx, y=wy, z=0.0)
            for wx, wy in world_points
        ]

        marker_array.markers.append(line_marker)

        if world_points:
            start_marker = Marker()
            start_marker.header.frame_id = "map"
            start_marker.header.stamp = self.get_clock().now().to_msg()
            start_marker.ns = "astar_start"
            start_marker.id = 1
            start_marker.type = Marker.SPHERE
            start_marker.action = Marker.ADD
            start_marker.pose.position = Point(
                x=world_points[0][0],
                y=world_points[0][1],
                z=0.0,
            )
            start_marker.scale.x = 0.2
            start_marker.scale.y = 0.2
            start_marker.scale.z = 0.2
            start_marker.color.b = 1.0
            start_marker.color.a = 1.0

            goal_marker = Marker()
            goal_marker.header.frame_id = "map"
            goal_marker.header.stamp = self.get_clock().now().to_msg()
            goal_marker.ns = "astar_goal"
            goal_marker.id = 2
            goal_marker.type = Marker.SPHERE
            goal_marker.action = Marker.ADD
            goal_marker.pose.position = Point(
                x=world_points[-1][0],
                y=world_points[-1][1],
                z=0.0,
            )
            goal_marker.scale.x = 0.2
            goal_marker.scale.y = 0.2
            goal_marker.scale.z = 0.2
            goal_marker.color.r = 1.0
            goal_marker.color.a = 1.0

            marker_array.markers.append(start_marker)
            marker_array.markers.append(goal_marker)

        self.marker_publisher.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = AstarPlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()