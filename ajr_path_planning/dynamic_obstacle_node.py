"""Mozgo akadaly szimulalasa es publikalasa."""
import rclpy
from rclpy.node import Node


class DynamicObstacleNode(Node):
    def __init__(self):
        super().__init__("dynamic_obstacle_node")
        self.get_logger().info("DynamicObstacleNode elindult.")
        # TODO: publisherek/subscriberek letrehozasa

    # TODO: callback fuggvenyek implementalasa


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
