# import torch
# print(torch.cuda.is_available())
# print(torch.cuda.get_device_name(0))


from ultralytics import YOLO

model = YOLO("../TRAIN/best.pt")

classes = model.names

print(classes)  