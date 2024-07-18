from ultralytics import YOLO

model = YOLO("yolov8n-pose.pt")

dataset_location = "/home/zeus/Área de Trabalho/TCC/yolov8_pose/dataset/data.yaml"

results = model.train(data=dataset_location, epochs=100, imgs=640)
