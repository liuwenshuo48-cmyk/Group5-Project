
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class GridPublisher(Node):

    def __init__(self):
        super().__init__('grid_publisher')

        self.pub = self.create_publisher(
            String,
            '/target_grid',
            10
        )

        self.timer = self.create_timer(
            1.0,
            self.publish_grid
        )

        self.current = "G1"


    def publish_grid(self):

        msg = String()
        msg.data = self.current

        self.pub.publish(msg)

        self.get_logger().info(
            "publish: " + msg.data
        )


def main():

    rclpy.init()

    node = GridPublisher()

    rclpy.spin(node)


if __name__ == "__main__":
    main()
