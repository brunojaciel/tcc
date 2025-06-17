from ultralytics import YOLO

#model = YOLO("yolo11n-pose.yaml")  # build a new model from YAML
#model = YOLO("yolo11n-pose.pt")  # load a pretrained model (recommended for training)
#model = YOLO("yolo11n-pose.yaml").load("yolo11n-pose.pt")  # build from YAML and transfer weights

model = YOLO("yolo11n.pt")

dataset_location = "/home/zeus/Área de Trabalho/TCC/yolov8_pose/dataset/Bottles/data.yaml"

results = model.train(data=dataset_location, epochs=100, imgsz=640)
