#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

import numpy as np

from yolov8_msgs.msg import InferenceResult
from yolov8_msgs.msg import Yolov8Inference

bridge = CvBridge()

def compute_angle(start, middle, end):
    vector1 = middle - start
    vector2 = end - middle

    magnitude_v1 = np.linalg.norm(vector1)
    magnitude_v2 = np.linalg.norm(vector2)

    if magnitude_v1 == 0 or magnitude_v2 == 0:
        return float('nan')

    dot_product = np.dot(vector1, vector2)
    cos_angle = dot_product / (magnitude_v1 * magnitude_v2)
    cos_angle = np.clip(cos_angle, -1.0, 1.0)

    angle_rad = np.arccos(cos_angle)
    angle_deg = np.degrees(angle_rad)
    
    return angle_deg

def calculate_new_point(pt1, pt2, factor=2.0):
    direction = np.array(pt2) - np.array(pt1)
    new_point = np.array(pt2) + direction * (factor - 1)
    return tuple(new_point.astype(int))

class YoloV8_Apont(Node):

    def __init__(self):
        super().__init__("yolov8_apont")

        self.model = YOLO('yolov8n-pose.pt')
        self.model_obj = YOLO('yolov8n.pt')
        self.active_keypoints = [6, 8, 10]
        self.scale = 1/2
        self.yolov8_inference = Yolov8Inference()

        topic_name = 'video_frames'

        self.subscription = self.create_subscription(Image, topic_name, self.analyze_pose, 10)

        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)

        #self.yolov8_apont_pub = self.create_publisher(PointData, "/Point_Data", 1)

    def analyze_pose(self, data):
        
        encoding = data.encoding
        if encoding == "bgr8":
            img = bridge.imgmsg_to_cv2(data, "bgr8")
        elif encoding == "rgb8":
            img = bridge.imgmsg_to_cv2(data, "rgb8")
        else:
            self.get_logger().error(f"Unsupported encoding: {encoding}")
            return
        
        height, width, _ = img.shape
        #window_width = int(img.shape[1] * self.scale)
        #window_height = int(img.shape[0] * self.scale)

        right_half = img[:, width // 2:]
        results = self.model(right_half)

        self.yolov8_inference.header.frame_id = "inference"
        self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()

        #self.point_data.header.frame_id = "inference"
        #self.point_data.header.stamp = self.get_clock().now().to_msg()

        # Iterate over detected people
        for result in results:
            #self.get_logger().info(f'Result')
            keypoints = result.keypoints.xy.cpu().numpy()
            
            for person_keypoints in keypoints:
                person_keypoints[:, 0] += width // 2  # Adjust keypoints to the full frame
                if person_keypoints.size == 0:
                    continue
                # Draw Keypoints and lines
                for i in range(len(self.active_keypoints) - 1):
                    if (self.active_keypoints[i] >= len(person_keypoints) or
                        self.active_keypoints[i + 1] >= len(person_keypoints)):
                        continue
                    
                    pt1 = tuple(person_keypoints[self.active_keypoints[i]].astype(int))
                    pt2 = tuple(person_keypoints[self.active_keypoints[i + 1]].astype(int))
                    #cv2.line(frame, pt1, pt2, color, 8)
                    #cv2.circle(frame, pt1, 5, color, -1)
                
                if len(person_keypoints) > max(self.active_keypoints):
                    angle = compute_angle(person_keypoints[self.active_keypoints[0]],
                                          person_keypoints[self.active_keypoints[1]],
                                          person_keypoints[self.active_keypoints[2]])
                    
                    if angle < 45:
                        results = self.model_obj(img, stream=True, verbose=False)
                        for r in results:
                            boxes = r.boxes
                            for box in boxes:
                                pt1 = person_keypoints[self.active_keypoints[1]].astype(int)
                                pt2 = person_keypoints[self.active_keypoints[2]].astype(int)
                                new_point_a = calculate_new_point(pt1, pt2, factor=2.0)

                                #self.infereimg_msgnce_result = UserPointing()

                                #Send the box to debug_no

                                #self.point_data.x = tupla[0]
                                #self.point_data.y = tupla[178]

                                #self.yolov8_apont_pub.publish(self.point_data)

                                new_point_b = calculate_new_point(pt1, pt2, factor=4.0)
                                new_point_c = calculate_new_point(pt1, pt2, factor=8.0)
                                x1, y1, x2, y2 = box.xyxy[0]

                                if (new_point_a[0] > x1 and new_point_a[0] < x2 and new_point_a[1] > y1 and new_point_a[1] < y2):
                                    #x1, y1, x2, y2 = box.xyxy[0]
                                    #x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                                    #w, h = x2 - x1, y2 - y1
                                    self.inference_result = InferenceResult()
                                    b = box.xyxy[0].to('cpu').detach().numpy().copy()  # get box coordinates in (top, left, bottom, right) format
                                    c = box.cls
                                    self.inference_result.class_name = self.model.names[int(c)]
                                    self.inference_result.top = int(b[0])
                                    self.inference_result.left = int(b[1])
                                    self.inference_result.bottom = int(b[2])
                                    self.inference_result.right = int(b[3])
                                    self.yolov8_inference.yolov8_inference.append(self.inference_result)

                                #self.get_logger().info(f'APONTAMENTO: {type(new_point_a)}')
                        self.yolov8_pub.publish(self.yolov8_inference)
                        self.yolov8_inference.yolov8_inference.clear()

    
    
    
class YoloV8_Pose(Node):

    def __init__(self):
        super().__init__("yolov8_pose")

        self.model = YOLO('/home/zeus/ros2_ws/src/my_robot_yolo/my_robot_yolo/runs/pose/train/weights/best.pt')

        self.yolov8_inference = Yolov8Inference()

        topic_name = 'video_frames'
        
        self.subscription = self.create_subscription(Image, topic_name, self.img_callback, 10)

        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
        self.img_pub = self.create_publisher(Image, "/inference_result", 1)


    def img_callback(self, data):
        # Log the encoding for debugging
        self.get_logger().info(f'Image encoding: {data.encoding}')
    
        encoding = data.encoding
        if encoding == "bgr8":
            img = bridge.imgmsg_to_cv2(data, "bgr8")
        elif encoding == "rgb8":
            img = bridge.imgmsg_to_cv2(data, "rgb8")
        else:
            self.get_logger().error(f"Unsupported encoding: {encoding}")
            return
    
        results = self.model(img)

        self.yolov8_inference.header.frame_id = "inference"
        self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()

        for r in results:
            boxes = r.boxes
            for box in boxes:
                self.inference_result = InferenceResult()
                b = box.xyxy[0].to('cpu').detach().numpy().copy()  # get box coordinates in (top, left, bottom, right) format
                c = box.cls
                self.inference_result.class_name = self.model.names[int(c)]
                self.inference_result.top = int(b[0])
                self.inference_result.left = int(b[1])
                self.inference_result.bottom = int(b[2])
                self.inference_result.right = int(b[3])
                self.yolov8_inference.yolov8_inference.append(self.inference_result)

            #camera_subscriber.get_logger().info(f"{self.yolov8_inference}")

        annotated_frame = results[0].plot()
        img_msg = bridge.cv2_to_imgmsg(annotated_frame)  

        self.img_pub.publish(img_msg)
        self.yolov8_pub.publish(self.yolov8_inference)
        self.yolov8_inference.yolov8_inference.clear()


def main(args=None):
    rclpy.init(args=args)
    #node = YoloV8_Apont()
    node = YoloV8_Pose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()