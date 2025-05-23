import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import sys
import select
import termios
import tty

class QueryImagePublisher(Node):
    def __init__(self):
        super().__init__('query_image_publisher')

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribes to the compressed image topic
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/mavic_1/decoded/out/compressed',
            self.image_callback,
            10
        )

        # Publisher to send latest image when spacebar is pressed
        self.image_pub = self.create_publisher(
            CompressedImage,
            '/query_image_rgb',
            reliable_qos
        )

        self.latest_image = None
        self.get_logger().info("Press spacebar to publish latest image...")

        # Setup terminal for non-blocking key input
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        # Check for keypress every 0.1 seconds
        self.timer = self.create_timer(0.1, self.check_keypress)

    def image_callback(self, msg):
        self.latest_image = msg

    def check_keypress(self):
        if select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            if key == ' ' and self.latest_image:
                self.get_logger().info("Spacebar pressed. Publishing image.")
                self.image_pub.publish(self.latest_image)

    def destroy_node(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = QueryImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
