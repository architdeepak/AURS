from ultralytics import YOLO

# Load a COCO-pretrained YOLOv3u model
model = YOLO("yolov3u.pt")

# Display model information (optional)
model.info()

# Train the model on the COCO8 example dataset for 100 epochs
results = model.train(data="coco8.yaml", epochs=100, imgsz=640)

# Run inference with the YOLOv3u model on the 'bus.jpg' image
results = model("image.png")