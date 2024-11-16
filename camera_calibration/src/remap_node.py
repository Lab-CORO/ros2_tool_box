import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalNode(Node):

    def __init__(self):
        super().__init__('minimal_node')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        pass  # Empty callback


def main(args=None):
    rclpy.init(args=args)
    minimal_node = MinimalNode()
    rclpy.spin(minimal_node)
    minimal_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
