# from ultralytics import YOLO

# def train_model():
#     model = YOLO("yolo8n.pt")  
#     results = model.train(data="./SUAS/SUAS/data.yaml", epochs=100, imgsz=640)

# if __name__ == '__main__':
#     train_model()

from ultralytics import YOLO

# Load a COCO-pretrained YOLOv8n model
def train():

    model = YOLO("yolov8n.pt")

    # Train the model on the COCO8 example dataset for 100 epochs
    results = model.train(data="./SUAS/SUAS/data.yaml", epochs=100, imgsz=640)

if __name__ == '__main__':
    train()
