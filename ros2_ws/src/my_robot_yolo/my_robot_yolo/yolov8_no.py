#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
from yolov8_msgs.msg import InferenceResult, Yolov8Inference

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

#Class responsible to inference the user's pointment
class YoloV8_Apont(Node):

    def __init__(self):
        super().__init__("yolov8_apont")

        self.model = YOLO('yolov8n-pose.pt')
        self.model_obj = YOLO('/home/zeus/ros2_ws/src/my_robot_yolo/my_robot_yolo/runs/detect_bot/train/weights/last.pt')
        self.active_keypoints = [6, 8, 10]
        self.scale = 1/2
        self.yolov8_inference = Yolov8Inference()

        topic_name = 'video_frames'

        self.subscription = self.create_subscription(Image, topic_name, self.analyze_pose, 10)
        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)

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
        right_half = img[:, width // 2:]
        results = self.model(right_half)

        self.yolov8_inference.header.frame_id = "inference"
        self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()

        for result in results:
            keypoints = result.keypoints.xy.cpu().numpy()
            
            for person_keypoints in keypoints:
                person_keypoints[:, 0] += width // 2  # Adjust keypoints to the full frame
                if person_keypoints.size == 0:
                    continue

                if len(person_keypoints) > max(self.active_keypoints):
                    angle = compute_angle(person_keypoints[self.active_keypoints[0]],
                                          person_keypoints[self.active_keypoints[1]],
                                          person_keypoints[self.active_keypoints[2]])
                    
                    if angle < 45:
                        results_obj = self.model_obj(img, stream=True, verbose=False)
                        for r in results_obj:
                            boxes = r.boxes
                            for box in boxes:
                                pt1 = person_keypoints[self.active_keypoints[1]].astype(int)
                                pt2 = person_keypoints[self.active_keypoints[2]].astype(int)
                                factor = 1.0

                                # Loop to find the desired condition
                                while factor <= 8.0:  # Maximum factor limit, can be adjusted as needed
                                    new_point = calculate_new_point(pt1, pt2, factor=factor)
                                    x1, y1, x2, y2 = box.xyxy[0]
                            
                                    if x1 <= new_point[0] <= x2 and y1 <= new_point[1] <= y2:
                                        self.inference_result = InferenceResult()
                                        b = box.xyxy[0].to('cpu').detach().numpy().copy()  # get box coordinates in (top, left, bottom, right) format
                                        c = box.cls
                                        self.inference_result.class_name = self.model_obj.names[int(c)]
                                        self.inference_result.top = int(b[0])
                                        self.inference_result.left = int(b[1])
                                        self.inference_result.bottom = int(b[2])
                                        self.inference_result.right = int(b[3])
                            
                                        if factor == 2.0:
                                            self.inference_result.category = 2
                                        elif factor == 4.0:
                                            self.inference_result.category = 3
                                        else:
                                            self.inference_result.category = 4
                            
                                        self.yolov8_inference.yolov8_inference.append(self.inference_result)
                                        break  # End the loop when the condition is met
                                    
                                    factor += 1.0  # Increase the factor
                        self.yolov8_pub.publish(self.yolov8_inference)
                        self.yolov8_inference.yolov8_inference.clear()

#Class responsible to inference the pose
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
                self.inference_result.category = 1
                self.yolov8_inference.yolov8_inference.append(self.inference_result)

        annotated_frame = results[0].plot()
        img_msg = bridge.cv2_to_imgmsg(annotated_frame)  

        self.img_pub.publish(img_msg)
        self.yolov8_pub.publish(self.yolov8_inference)
        self.yolov8_inference.yolov8_inference.clear()

#Class responsible to inference the user's pointment and pose
class YoloV8(Node):
    def __init__(self):
        super().__init__("yolov8")

        self.model = YOLO('/home/zeus/ros2_ws/src/my_robot_yolo/my_robot_yolo/runs/pose/train/weights/best.pt')
        self.model_obj = YOLO('/home/zeus/ros2_ws/src/my_robot_yolo/my_robot_yolo/runs/detect_bot/train/weights/last.pt')
        self.active_keypoints = [6, 8, 10]
        self.scale = 1/2
        self.yolov8_inference = Yolov8Inference()

        topic_name = 'video_frames'

        self.subscription = self.create_subscription(Image, topic_name, self.analyze_pose, 10)
        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
        

    def analyze_pose(self, data):
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
                self.inference_result.category = 1
                self.yolov8_inference.yolov8_inference.append(self.inference_result)

                #analyze_pointment
                height, width, _ = img.shape
                right_half = img[:, width // 2:]
                results = self.model(right_half)

                for result in results:
                    keypoints = result.keypoints.xy.cpu().numpy()

                    for person_keypoints in keypoints:
                        person_keypoints[:, 0] += width // 2  # Adjust keypoints to the full frame
                        if person_keypoints.size == 0:
                            continue
                        
                        if len(person_keypoints) > max(self.active_keypoints):
                            angle = compute_angle(person_keypoints[self.active_keypoints[0]],
                                                  person_keypoints[self.active_keypoints[1]],
                                                  person_keypoints[self.active_keypoints[2]])

                            if angle < 45:
                                results_obj = self.model_obj(img, stream=True, verbose=False)
                                for r in results_obj:
                                    boxes = r.boxes
                                    for box in boxes:
                                        pt1 = person_keypoints[self.active_keypoints[1]].astype(int)
                                        pt2 = person_keypoints[self.active_keypoints[2]].astype(int)
                                        factor = 1.0

                                        # Loop to find the desired condition
                                        while factor <= 8.0:  # Maximum factor limit, can be adjusted as needed
                                            new_point = calculate_new_point(pt1, pt2, factor=factor)
                                            x1, y1, x2, y2 = box.xyxy[0]

                                            if x1 <= new_point[0] <= x2 and y1 <= new_point[1] <= y2:
                                                self.inference_result = InferenceResult()
                                                b = box.xyxy[0].to('cpu').detach().numpy().copy()  # get box coordinates in (top, left, bottom, right) format
                                                c = box.cls
                                                self.inference_result.class_name = self.model_obj.names[int(c)]
                                                self.inference_result.top = int(b[0])
                                                self.inference_result.left = int(b[1])
                                                self.inference_result.bottom = int(b[2])
                                                self.inference_result.right = int(b[3])

                                                if factor == 2.0:
                                                    self.inference_result.category = 2
                                                elif factor == 4.0:
                                                    self.inference_result.category = 3
                                                else:
                                                    self.inference_result.category = 4

                                                self.yolov8_inference.yolov8_inference.append(self.inference_result)
                                                break  # End the loop when the condition is met
                                            
                                            factor += 1.0  # Increase the factor
    
        self.yolov8_pub.publish(self.yolov8_inference)
        self.yolov8_inference.yolov8_inference.clear()


def main(args=None):
    rclpy.init(args=args)

    yolo_apont = YoloV8_Apont()
    yolo_pose = YoloV8_Pose()
    #yolo = YoloV8()

    # Use MultiThreadedExecutor to handle multiple nodes in parallel
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(yolo_apont)
    executor.add_node(yolo_pose)
    #executor.add_node(yolo)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        yolo_apont.destroy_node()
        yolo_pose.destroy_node()
        #yolo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
