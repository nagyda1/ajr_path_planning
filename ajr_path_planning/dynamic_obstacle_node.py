import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, PoseArray
from visualization_msgs.msg import Marker


class DynamicObstacleNode(Node):
    def __init__(self):
        super().__init__("dynamic_obstacle_node")

        self.positions = [
            (3.25, 3.25),
            (3.75, 3.25),
            (4.25, 3.25),
            (4.75, 3.25),
            (4.75, 3.75),
            (4.25, 3.75),
            (3.75, 3.75),
            (3.25, 3.75),
        ]
        self.position_index = 0

        self.obstacle_publisher = self.create_publisher(
            PoseArray,
            "/dynamic_obstacles",
            10,
        )

        self.marker_publisher = self.create_publisher(
            Marker,
            "/dynamic_obstacle/marker",
            10,
        )

        self.timer = self.create_timer(4.0, self.publish_obstacle)

        self.get_logger().info(
            "DynamicObstacleNode elindult. A mozgó akadály 4 másodpercenként pozíciót vált."
        )

    def publish_obstacle(self):
        x, y = self.positions[self.position_index]

        pose_array = PoseArray()
        pose_array.header.frame_id = "map"
        pose_array.header.stamp = self.get_clock().now().to_msg()

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.orientation.w = 1.0

        pose_array.poses.append(pose)
        self.obstacle_publisher.publish(pose_array)

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = pose_array.header.stamp
        marker.ns = "dynamic_obstacle"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position = Point(x=x, y=y, z=0.3)
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.5
        marker.scale.y = 1.5
        marker.scale.z = 0.5
        marker.color.r = 0.8
        marker.color.g = 0.0
        marker.color.b = 0.8
        marker.color.a = 0.85

        self.marker_publisher.publish(marker)

        self.get_logger().info(
            f"Dinamikus akadály új pozíciója: ({x:.2f}, {y:.2f})"
        )

        self.position_index = (
            self.position_index + 1
        ) % len(self.positions)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
