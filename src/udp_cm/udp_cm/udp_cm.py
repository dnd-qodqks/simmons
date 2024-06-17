import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from result_msgs.msg import Bbox
from cv_bridge import CvBridge, CvBridgeError

import socket
import cv2
import numpy as np
import time
import base64
import sys
from datetime import datetime

class ImageSubscriberUDPSenderReceiver(Node):
    def __init__(self):
        super().__init__('image_subscriber_udp_sender_receiver')
        self.img_sub_ = self.create_subscription(
            Image,
            '/robot/D435/color/image_raw',  # 구독할 이미지 토픽 이름
            self.image_callback,
            10)
        
        self.bbox_pub_ = self.create_publisher(Bbox, '/bbox', 10)
        
        self.bridge = CvBridge()
        
        self.pre_xyxy = [0, 0, 0, 0]
        
        self.get_logger().info("Start UDP Node!!")
        
        #self.TCP_SERVER_IP = '192.168.2.158'
        self.TCP_SERVER_IP = '192.168.244.26'
        self.TCP_SERVER_PORT = 5001
        self.connectCount = 0
        self.connectServer()
        
    def connectServer(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.TCP_SERVER_IP, self.TCP_SERVER_PORT))
            self.get_logger().info(u'Client socket is connected with Server socket [ TCP_SERVER_IP: ' + self.TCP_SERVER_IP + ', TCP_SERVER_PORT: ' + str(self.TCP_SERVER_PORT) + ' ]')
            self.connectCount = 0
        except Exception as e:
            self.get_logger().info(e)
            self.get_logger().info(f'{self.connectCount} times try to connect with server')
            self.sock.close()
            time.sleep(1)
            self.connectServer()

    def recvall(self, sock, count):
        buf = b''
        while count:
            newbuf = sock.recv(count)
            if not newbuf: return None
            buf += newbuf
            count -= len(newbuf)
        return buf

    def image_callback(self, msg):
        try:
            # ROS 이미지 메시지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            encode_param=[int(cv2.IMWRITE_JPEG_QUALITY),90]
            result, imgencode = cv2.imencode('.jpg', cv_image, encode_param)
            data = np.array(imgencode)
            stringData = base64.b64encode(data)
            length = str(len(stringData))
            self.sock.sendall(length.encode('utf-8').ljust(64))
            self.sock.send(stringData)
            
            #############
            
            length = self.recvall(self.sock, 64)
            if length is not None:
                length1 = length.decode('utf-8')
                stringData = self.recvall(self.sock, int(length1))
                if stringData is not None:
                    decoded_data = base64.b64decode(stringData)
                    response = decoded_data.decode('utf-8')
                
                    x1, y1, x2, y2, flag = map(int, response.split(','))
                    self.get_logger().debug(f'Received response) x1: {x1}, y1: {y1}, x2: {x2}, y2: {y2}, flag: {flag}')
                    
                    if flag:
                        self.pre_xyxy = [x1, y1, x2, y2]
                        
                        msg = Bbox()
                        msg.x1 = x1
                        msg.y1 = y1
                        msg.x2 = x2
                        msg.y2 = y2
                        self.bbox_pub_.publish(msg)
                    else:
                        msg = Bbox()
                        msg.x1 = self.pre_xyxy[0]
                        msg.y1 = self.pre_xyxy[1]
                        msg.x2 = self.pre_xyxy[2]
                        msg.y2 = self.pre_xyxy[3]
                        self.bbox_pub_.publish(msg)

        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge 변환 오류: {e}')
        except self.sock.timeout:
            self.get_logger().warn('서버로부터 응답 시간 초과')
        except Exception as e:
            self.get_logger().warn(f'{e}')

def main(args=None):
    rclpy.init(args=args)
    image_subscriber_udp_sender_receiver = ImageSubscriberUDPSenderReceiver()
    rclpy.spin(image_subscriber_udp_sender_receiver)
    image_subscriber_udp_sender_receiver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

