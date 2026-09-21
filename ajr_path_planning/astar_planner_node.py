"""A* alapu utvonaltervezo node, MarkerArray vizualizacioval."""
import rclpy
from rclpy.node import Node


class AstarPlannerNode(Node):
    def __init__(self):
        super().__init__("astar_planner_node")
        self.get_logger().info("AstarPlannerNode elindult.")
        # TODO: publisherek/subscriberek letrehozasa

    # TODO: callback fuggvenyek implementalasa


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
