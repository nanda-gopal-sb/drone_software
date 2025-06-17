import subprocess
import cv2
import numpy as np
from ultralytics import YOLO

# Load YOLO model
model = YOLO("yolov8n.pt")

# FFmpeg command to read from your input (e.g., RTSP, file, etc.)
# Replace "input_source" with your actual source (e.g., rtsp://..., video.mp4, etc.)
ffmpeg_cmd = [
    'ffmpeg',
    '-i', 'input_source',        # <- change this
    '-loglevel', 'quiet',
    '-an',                       # no audio
    '-f', 'rawvideo',
    '-pix_fmt', 'bgr24',
    '-vcodec', 'rawvideo',
    '-'
]

# Open FFmpeg as a subprocess
process = subprocess.Popen(ffmpeg_cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)

# Set the frame size according to the stream
width = 640
height = 480
frame_size = width * height * 3  # bgr24 = 3 bytes per pixel

while True:
    raw_frame = process.stdout.read(frame_size)
    if not raw_frame:
        break

    # Convert raw bytes to numpy array
    frame = np.frombuffer(raw_frame, np.uint8).reshape((height, width, 3))

    # Run YOLO
    results = model(frame)

    # Visualize
    annotated = results[0].plot()
    cv2.imshow('YOLO on FFmpeg Pipe', annotated)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

process.stdout.close()
process.wait()
cv2.destroyAllWindows()
