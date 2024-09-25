#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class MyNode(Node):

    def __init__(self):
        super().__init__("camera")
        #self.get_logger().info("Hello from ROS2")

        topic_name = 'video_frames'

        self.publisher_ = self.create_publisher(Image, topic_name, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.cap = cv2.VideoCapture(0)
        self.br = CvBridge()


    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret == True:
            # Convert frame to ROS Image message with encoding 'bgr8'
            self.publisher_.publish(self.br.cv2_to_imgmsg(frame, encoding='bgr8'))
        self.get_logger().info('Publishing video frame')



def main(args=None):
    rclpy.init(args=args)
    
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
