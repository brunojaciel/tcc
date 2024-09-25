#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading

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

        # Publisher for the processed image
        #self.img_pub = self.create_publisher(Image, "/inference_result_cv2", 1)

        self.lock = threading.Lock()
        self.image_display_thread = threading.Thread(target=self.display_image, daemon=True)
        self.image_display_thread.start()

    def camera_callback(self, data):
        with self.lock:
            self.image_provider = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def yolo_callback(self, data):
        with self.lock:
            img = self.image_provider
            if img is None:
                self.get_logger().warn("No image data available")
                return

            for r in data.yolov8_inference:
                class_name = r.class_name
                top = r.top
                left = r.left
                bottom = r.bottom
                right = r.right
                self.get_logger().info(f"{self.cnt} {class_name} : {top}, {left}, {bottom}, {right}")

                # Draw rectangle on the image
                cv2.rectangle(img, (left, top), (right, bottom), (255, 255, 0), 2)
                self.cnt += 1

            self.cnt = 0

            # Store image for display
            self.image_for_display = img

    def display_image(self):
        while rclpy.ok():
            if hasattr(self, 'image_for_display'):
                cv2.imshow("camera", self.image_for_display)
                cv2.waitKey(1)
            cv2.waitKey(100)  # Prevent high CPU usage

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

if __name__ == '__main__':
    main()
