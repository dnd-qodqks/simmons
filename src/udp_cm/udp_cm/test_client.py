import rclpy
from rclpy.node import Node
import socket

class ClientNode(Node):
    def __init__(self):
        super().__init__('client_node')
        self.server_ip = '192.168.2.158'  # 서버의 IP 주소 (로컬호스트)
        self.server_port = 5001
        self.data = b"Hello, Server!"
        
        self.timer = self.create_timer(1.0, self.send_data)

    def send_data(self):
        try:
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.connect((self.server_ip, self.server_port))
            client_socket.sendall(self.data)
            client_socket.close()
            self.get_logger().info("Data sent.")
        except Exception as e:
            self.get_logger().error(f"Failed to send data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ClientNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

