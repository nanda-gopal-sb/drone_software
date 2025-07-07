from ultralytics import YOLO

# Load a pretrained YOLO11n model
model = YOLO("E:/TRAIN/best.pt")

# Define path to video file
#source = "rtsp://192.168.144.25:8554/main.264"
source = "E:/test_dataset_2/test_dataset/car"

# Run inference on the source
results = model(source)  # generator of Results objects


for result in results:
    boxes = result.boxes  # Boxes object for bounding box outputs
    masks = result.masks  # Masks object for segmentation masks outputs
    keypoints = result.keypoints  # Keypoints object for pose outputs
    probs = result.probs  # Probs object for classification outputs
    obb = result.obb
    result.show()# Oriented boxes object for OBB outputs

