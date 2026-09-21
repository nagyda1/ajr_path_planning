"""Statikus/generalt terkep publikalasa OccupancyGrid formaban."""
import rclpy
from rclpy.node import Node


class MapServerNode(Node):
    def __init__(self):
        super().__init__("map_server_node")
        self.get_logger().info("MapServerNode elindult.")
        # TODO: publisherek/subscriberek letrehozasa

    # TODO: callback fuggvenyek implementalasa


def main(args=None):
    rclpy.init(args=args)
    node = MapServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
