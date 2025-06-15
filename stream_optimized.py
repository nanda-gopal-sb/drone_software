import cv2
from ultralytics import YOLO
import os

# Set OpenCV FFMPEG capture options for low latency
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;udp|fflags;nobuffer|flags;low_delay|probesize;32|analyzeduration;0|sync;skip_frames"
model = YOLO("best.pt")

source = "rtsp://192.168.144.25:8554/main.264"
#source = "http://localhost:3000/mjpeg-stream"
print("Starting stream processing")

# The YOLO model's stream processing will now use the configured FFMPEG options
results = model(source, stream=True, verbose=False)

for result in results:
    annotated_frame = result.plot()
    cv2.imshow("YOLO Real-time Detection", annotated_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
print("Stream processing finished.")
