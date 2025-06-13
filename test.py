from ultralytics import YOLO

# Load a pretrained YOLO11n model
model = YOLO("../yolo11n.pt")

# Define path to video file
source = "rtsp://192.168.144.25:8554/main.264"


# Run inference on the source
results = model(source, stream=True)  # generator of Results objects


for result in results:
    boxes = result.boxes  # Boxes object for bounding box outputs
    masks = result.masks  # Masks object for segmentation masks outputs
    keypoints = result.keypoints  # Keypoints object for pose outputs
    probs = result.probs  # Probs object for classification outputs
    obb = result.obb  # Oriented boxes object for OBB outputs

