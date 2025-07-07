import cv2
from ultralytics import YOLO
import os
import time # For basic FPS measurement and potential debug

# Explanation of flags:
# - rtsp_transport;tcp: Forces TCP for RTSP data.
# - fflags;nobuffer: Disables input buffering.
# - flags;low_delay: Hints to the decoder to prioritize low delay.
# - probesize;32: Reduces the data size used for stream analysis.
# - analyzeduration;0: Reduces the time used for stream analysis (to 0 microseconds).
# - sync;skip_frames: Tells FFmpeg to skip frames if decoding falls behind,
#                     prioritizing real-time playback over showing every frame.
# - rw_timeout;5000000: (Optional, but often useful) Read/write timeout in microseconds.
#                       Helps prevent hangs on network issues. 5 seconds here.
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp|fflags;nobuffer|flags;low_delay|probesize;32|analyzeduration;0|sync;skip_frames|rw_timeout;5000000"

model = YOLO("best.pt")

source = "rtsp://192.168.144.25:8554/main.264"

print("Starting stream processing...")

results = model(source, stream=True, verbose=False)

start_time = time.time()
frame_count = 0

for result in results:
    frame_count += 1
    annotated_frame = result.plot() # YOLO's method to draw detections on the frame

    cv2.imshow("YOLO Real-time Detection", annotated_frame)

    # Calculate and print basic FPS every 30 frames
    if frame_count % 30 == 0:
        end_time = time.time()
        fps = 30 / (end_time - start_time)
        print(f"Processed 30 frames. Current FPS: {fps:.2f}")
        start_time = time.time() # Reset for next interval

    # Exit condition
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
print("Stream processing finished.")
