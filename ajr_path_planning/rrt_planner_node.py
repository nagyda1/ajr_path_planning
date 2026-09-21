"""RRT/RRT* alapu utvonaltervezo node kinematikai korlatokkal."""
import rclpy
from rclpy.node import Node


class RrtPlannerNode(Node):
    def __init__(self):
        super().__init__("rrt_planner_node")
        self.get_logger().info("RrtPlannerNode elindult.")
        # TODO: publisherek/subscriberek letrehozasa

    # TODO: callback fuggvenyek implementalasa


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
