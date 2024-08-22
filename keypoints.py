from ultralytics import YOLO
import cv2
import numpy as np
import matplotlib.pyplot as plt

def compute_angle(start, middle, end):
    vector1 = middle - start
    vector2 = end - middle

    magnitude_v1 = np.linalg.norm(vector1)
    magnitude_v2 = np.linalg.norm(vector2)

    # Check for zero magnitude to avoid division by zero
    if magnitude_v1 == 0 or magnitude_v2 == 0:
        return float('nan')  # Return NaN to indicate invalid angle

    dot_product = np.dot(vector1, vector2)
    cos_angle = dot_product / (magnitude_v1 * magnitude_v2)

    # Ensure cos_angle is in the valid range for acos
    cos_angle = np.clip(cos_angle, -1.0, 1.0)

    angle_rad = np.arccos(cos_angle)
    angle_deg = np.degrees(angle_rad)
    
    return angle_deg


class PoseEstimation:
    def __init__(self, camera_index=0):
        self.model = YOLO('yolov8n-pose.pt')
        self.active_keypoints = [6, 8, 10]
        #
        self.camera_index = camera_index
        self.scale = 1/2
        current_fps = 24
        desired_fps = 10
        self.skip_factor = current_fps // desired_fps

    def analyze_pose(self, show_angle=False):
        if show_angle:
            plt.ion()
            #fig, ax = plt.subplots()
            angle = 0
            #angles = []
            #time = []

        frame_count = 0
        color = (255, 255, 0)

        cv2.namedWindow("KeyPoints on Video", cv2.WINDOW_NORMAL)
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
            keypoints = results[0].keypoints.xy.cpu().numpy()[0]
            keypoints[:, 0] += width // 2

            # Check if keypoints are detected
            if keypoints.size == 0:
                print("No keypoints detected")
                continue

            # Draw Keypoints
            for i in range(len(self.active_keypoints) - 1):
                if self.active_keypoints[i] >= len(keypoints) or self.active_keypoints[i + 1] >= len(keypoints):
                    continue  # Skip if keypoints are not in range
                pt1 = tuple(keypoints[self.active_keypoints[i]].astype(int))
                pt2 = tuple(keypoints[self.active_keypoints[i + 1]].astype(int))
                cv2.line(frame, pt1, pt2, color, 8)
                cv2.circle(frame, pt1, 5, color, -1)

            if show_angle:
                if len(keypoints) > max(self.active_keypoints):
                    angle = compute_angle(keypoints[self.active_keypoints[0]],
                                          keypoints[self.active_keypoints[1]],
                                          keypoints[self.active_keypoints[2]])
                    
                    #angle<45
                    if np.isnan(angle):
                        angle_str = "N/A"
                    else:
                        angle_str = str(round(angle))

                    cv2.putText(frame, angle_str, (270, 1500),
                                cv2.FONT_HERSHEY_SIMPLEX, 5, color, 5, cv2.LINE_AA)

                    print("Angle: " + angle_str)
                else:
                    print("Not enough keypoints detected for angle calculation")

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
