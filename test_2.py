import cv2
from ultralytics import YOLO
import os
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;udp|fflags;nobuffer|flags;low_delay|probesize;32|analyzeduration;0"

model = YOLO("../yolo11n.pt")

source = "rtsp://192.168.144.25:8554/main.264"

print("Starting stream processing")

results = model(source, stream=True, verbose=False)

for result in results:
    annotated_frame = result.plot()
    cv2.imshow("YOLO Real-time Detection", annotated_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
print("Stream processing finished.")

