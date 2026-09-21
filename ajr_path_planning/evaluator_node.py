"""A* es RRT teljesitmeny-osszehasonlitas (futasido, utvonalhossz, sikeresseg)."""
import rclpy
from rclpy.node import Node


class EvaluatorNode(Node):
    def __init__(self):
        super().__init__("evaluator_node")
        self.get_logger().info("EvaluatorNode elindult.")
        # TODO: publisherek/subscriberek letrehozasa

    # TODO: callback fuggvenyek implementalasa


def main(args=None):
    rclpy.init(args=args)
    node = EvaluatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
