import rclpy
from rclpy.node import Node
from result_msgs.msg import Person

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('test_person_sub')
        self.subscription = self.create_subscription(
            Person,
            '/person_info',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f"length: {msg.length}, degree: {msg.degree}")


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()