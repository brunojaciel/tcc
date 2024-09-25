#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
import numpy as np  # Import numpy for handling image arrays

from yolov8_msgs.msg import Yolov8Inference

class YoloSubscriber(Node):
    def __init__(self):
        super().__init__('debug_no')

        self.bridge = CvBridge()
        self.image_provider = None
        self.cnt = 0

        topic_name = 'video_frames'

        # Subscribe to the image topic
        self.subscription_image = self.create_subscription(
            Image,
            topic_name,
            self.camera_callback,
            10
        )

        # Subscribe to the Yolov8 inference topic
        self.subscription_yolo = self.create_subscription(
            Yolov8Inference,
            '/Yolov8_Inference',
            self.yolo_callback,
            10
        )

        self.image_for_display = None
        self.display_thread = threading.Thread(target=self.display_image, daemon=True)
        self.display_thread.start()

    def camera_callback(self, data):
        self.image_provider = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def yolo_callback(self, data):
        if self.image_provider is None:
            self.get_logger().warn("No image data available")
            return

        img = self.image_provider.copy()  # Work on a copy of the image

        for r in data.yolov8_inference:
            class_name = r.class_name
            top = r.top
            left = r.left
            bottom = r.bottom
            right = r.right
            category = r.category
            self.get_logger().info(f"{self.cnt} {class_name}, {category} : {top}, {left}, {bottom}, {right}")

            color_map = {
                1: (0, 255, 255),
                2: (0, 0, 255),
                3: (255, 0, 0)
            }

            color = color_map.get(category, (255, 255, 0))
            # Draw rectangle on the image
            cv2.rectangle(img, (top, left), (bottom, right), color)
            self.cnt += 1

        self.cnt = 0

        # Store image for display
        self.image_for_display = img

    def display_image(self):
        while rclpy.ok():
            if self.image_for_display is not None:
                cv2.imshow('Yolo Inference', self.image_for_display)
                cv2.waitKey(1)  # Refresh the window
            else:
                cv2.waitKey(100)  # Wait if no image is available

def main(args=None):
    rclpy.init(args=args)
    
    yolo_subscriber = YoloSubscriber()
    
    # Use a MultiThreadedExecutor to manage multiple nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(yolo_subscriber)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    rate = yolo_subscriber.create_rate(2)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass

    # Properly destroy nodes
    yolo_subscriber.destroy_node()
    
    rclpy.shutdown()
    executor_thread.join()
    cv2.destroyAllWindows()  # Close OpenCV windows

if __name__ == '__main__':
    main()
