from ultralytics import YOLO


# Load a pretrained YOLO model (recommended for training)
model = YOLO("yolo11n.pt")

# Train the model using the 'coco8.yaml' dataset for 3 epochs
results = model.train(data="/home/esteb37/ros2_ws/src/dextrous_hand/dextrous_hand/yolo/model.yaml", imgsz = 224)

# Evaluate the model's performance on the validation set
results = model.val()

# Export the model to ONNX format
success = model.export(format="onnx")