import rclpy
from rclpy.node import Node
from result_msgs.msg import Person

class PublisherNode(Node):

    def __init__(self):
        super().__init__('test_person_pub')
        self.publisher_ = self.create_publisher(Person, '/person_info', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Person()
        msg.length = 0.5
        msg.degree = 120.0
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    publisher_node = PublisherNode()

    rclpy.spin(publisher_node)

    publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
