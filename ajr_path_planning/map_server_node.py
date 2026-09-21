import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion

WIDTH = 20
HEIGHT = 20
RESOLUTION = 0.5


class MapServerNode(Node):
    def __init__(self):
        super().__init__("map_server_node")

        self.grid_data = self.build_static_grid()
        self.publisher = self.create_publisher(OccupancyGrid, "/map", 10)
        self.timer = self.create_timer(1.0, self.publish_map)

        self.get_logger().info(
            f"MapServerNode elindult: {WIDTH}x{HEIGHT}, "
            f"{RESOLUTION} m/cella felbontás."
        )

    def build_static_grid(self):
        grid = [0] * (WIDTH * HEIGHT)

        def set_cell(x, y, value):
            if 0 <= x < WIDTH and 0 <= y < HEIGHT:
                grid[y * WIDTH + x] = value

        for x in range(WIDTH):
            set_cell(x, 0, 100)
            set_cell(x, HEIGHT - 1, 100)

        for y in range(HEIGHT):
            set_cell(0, y, 100)
            set_cell(WIDTH - 1, y, 100)

        for y in range(4, 14):
            set_cell(10, y, 100)

        for x in range(3, 8):
            set_cell(x, 15, 100)

        return grid

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        msg.info.resolution = RESOLUTION
        msg.info.width = WIDTH
        msg.info.height = HEIGHT
        msg.info.origin = Pose(
            position=Point(x=0.0, y=0.0, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        )

        msg.data = self.grid_data
        self.publisher.publish(msg)


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