import rclpy
from rclpy.node import Node
import socket
import threading

class SocketReceiver(Node):
    def __init__(self):
        super().__init__('socket_receiver')
        self.server_port = 5001
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind(('0.0.0.0', self.server_port))
        self.server_socket.listen(1)
        self.get_logger().info(f"Listening on port {self.server_port}...")

        self.thread = threading.Thread(target=self.receive_data)
        self.thread.start()

    def receive_data(self):
        conn, addr = self.server_socket.accept()
        self.get_logger().info(f"Connection from {addr} has been established.")
        
        data = b""
        while True:
            packet = conn.recv(4096)
            if not packet:
                break
            data += packet
        
        conn.close()
        self.get_logger().info("Data received.")
        self.get_logger().info(f"Received data: {data.decode('utf-8')}")  # Assuming the data is text

def main(args=None):
    rclpy.init(args=args)
    socket_receiver = SocketReceiver()
    rclpy.spin(socket_receiver)
    socket_receiver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

