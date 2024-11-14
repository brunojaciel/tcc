from ultralytics import YOLO

model = YOLO("yolov8n-pose.pt")

dataset_location = "/home/zeus/Área de Trabalho/TCC/yolov8_pose/dataset/New_poses/data.yaml"

results = model.train(data=dataset_location, epochs=100, imgsz=640)
