from ultralytics import YOLO
import cv2
import numpy as np
import matplotlib.pyplot as plt
import cvzone
import math

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

class PoseEstimation:
    def __init__(self, camera_index=0):
        self.model = YOLO('yolov8n-pose.pt')
        self.model_obj = YOLO('yolov8n.pt')
        self.active_keypoints = [6, 8, 10]
        self.camera_index = camera_index
        self.scale = 1/2
        current_fps = 24
        desired_fps = 10
        self.skip_factor = current_fps // desired_fps

    def show_box(self, frame, x1, x2, y1, y2, box, new_point, w, h, color):
        if (x1 <= new_point[0] <= x2) and (y1 <= new_point[1] <= y2):
            cvzone.cornerRect(frame, (x1, y1, w, h), colorC=color)
            conf = math.ceil((box.conf[0] * 100)) / 100
            cls = int(box.cls[0])
            cvzone.putTextRect(frame, f'{cls} {conf}', (max(0, x1), max(35, y1)), scale=1)

    def analyze_pose(self, show_angle=False):
        if show_angle:
            plt.ion()
            angle = 0

        frame_count = 0
        color = (255, 255, 0)

        cv2.namedWindow("KeyPoints on Video", cv2.WINDOW_AUTOSIZE)
        cap = cv2.VideoCapture(self.camera_index)

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break
            frame_count += 1
            if frame_count % self.skip_factor != 0:
                continue

            height, width, _ = frame.shape
            window_width = int(frame.shape[1] * self.scale)
            window_height = int(frame.shape[0] * self.scale)
            cv2.resizeWindow("KeyPoints on Video", window_width, window_height)

            right_half = frame[:, width // 2:]
            results = self.model(right_half)
            
            # Iterate over detected people
            for result in results:
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
                        cv2.line(frame, pt1, pt2, color, 8)
                        cv2.circle(frame, pt1, 5, color, -1)

                    if show_angle:
                        if len(person_keypoints) > max(self.active_keypoints):
                            angle = compute_angle(person_keypoints[self.active_keypoints[0]],
                                                  person_keypoints[self.active_keypoints[1]],
                                                  person_keypoints[self.active_keypoints[2]])
                            
                            if angle < 45:
                                print('Apontamento!')
                                results = self.model_obj(frame, stream=True, verbose=False)
                                for r in results:
                                    boxes = r.boxes
                                    for box in boxes:
                                        x1, y1, x2, y2 = box.xyxy[0]
                                        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                                        w, h = x2 - x1, y2 - y1
                                        
                                        pt1 = person_keypoints[self.active_keypoints[1]].astype(int)
                                        pt2 = person_keypoints[self.active_keypoints[2]].astype(int)
                                        new_point_a = calculate_new_point(pt1, pt2, factor=2.0)
                                        new_point_b = calculate_new_point(pt1, pt2, factor=4.0)
                                        new_point_c = calculate_new_point(pt1, pt2, factor=8.0)
                                        
                                        color_a = (64, 224, 208)
                                        color_b = (50, 205, 50)
                                        color_c = (255, 127, 80)

                                        self.show_box(frame, x1, x2, y1, y2, box, new_point_a, w, h, color_a)
                                        self.show_box(frame, x1, x2, y1, y2, box, new_point_b, w, h, color_b)
                                        self.show_box(frame, x1, x2, y1, y2, box, new_point_c, w, h, color_c)
                    
            cv2.imshow("KeyPoints on Video", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()

def run_analyze_pose(show_angle):
    pe = PoseEstimation(camera_index=0)  # Use 0 for the default webcam
    pe.analyze_pose(show_angle=show_angle)
    
if __name__ == '__main__':
    run_analyze_pose(show_angle=True)
