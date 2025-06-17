import cv2
from ultralytics import YOLO
import os

# Set OpenCV FFMPEG capture options for low latency
# These flags are still useful for the initial capture, even if we manually process frames.
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;udp|fflags;nobuffer|flags;low_delay|probesize;32|analyzeduration;0"

model = YOLO("../yolo11n.pt")
source = "2"

print("Starting stream processing with digital zoom...")

# --- Digital Zoom Parameters ---
# Initial zoom factor (1.0 means no zoom, 2.0 means 2x zoom, etc.)
zoom_factor = 1.0
# Initial crop center (normalized coordinates 0.0 to 1.0)
# 0.5, 0.5 means center of the image
zoom_center_x = 0.5
zoom_center_y = 0.5

# --- OpenCV Video Capture ---
cap = cv2.VideoCapture(source)

if not cap.isOpened():
    print(f"Error: Could not open video stream from {source}")
    exit()

# Get original frame dimensions
original_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
original_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

print(f"Original Stream Resolution: {original_width}x{original_height}")

while True:
    ret, frame = cap.read()
    if not ret:
        print("End of stream or error reading frame.")
        break

    # --- Apply Digital Zoom ---
    if zoom_factor > 1.0:
        # Calculate the size of the cropped region
        cropped_width = int(original_width / zoom_factor)
        cropped_height = int(original_height / zoom_factor)

        # Calculate top-left corner of the cropped region based on zoom_center
        x1 = int(zoom_center_x * original_width - cropped_width / 2)
        y1 = int(zoom_center_y * original_height - cropped_height / 2)

        # Ensure crop coordinates are within bounds
        x1 = max(0, x1)
        y1 = max(0, y1)

        x2 = x1 + cropped_width
        y2 = y1 + cropped_height

        # Adjust if the calculated crop extends beyond the frame boundaries
        if x2 > original_width:
            x1 = original_width - cropped_width
            x2 = original_width
        if y2 > original_height:
            y1 = original_height - cropped_height
            y2 = original_height

        # Perform the crop
        cropped_frame = frame[y1:y2, x1:x2]

        # Resize the cropped frame back to the original display size (or desired display size)
        # This gives the "zoom" effect
        processed_frame = cv2.resize(cropped_frame, (original_width, original_height), interpolation=cv2.INTER_LINEAR)
    else:
        # No zoom, use the original frame
        processed_frame = frame

    # --- Perform YOLO Inference on the processed (zoomed) frame ---
    # Note: If you zoom in significantly, objects might appear larger
    # and fewer might be in the frame.
    results = model(processed_frame, stream=False, verbose=False) # stream=False as we process frame by frame

    # results is a list, even for a single frame, so iterate or take the first element
    if results:
        annotated_frame = results[0].plot()
    else:
        annotated_frame = processed_frame # Fallback if no detections or issue

    cv2.imshow("YOLO Real-time Detection with Zoom", annotated_frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('z'): # Increase zoom
        zoom_factor += 0.1
        if zoom_factor > 4.0: # Max zoom 4x
            zoom_factor = 4.0
        print(f"Zoom factor: {zoom_factor:.1f}x")
    elif key == ord('x'): # Decrease zoom
        zoom_factor -= 0.1
        if zoom_factor < 1.0: # Min zoom 1x (no zoom)
            zoom_factor = 1.0
        print(f"Zoom factor: {zoom_factor:.1f}x")
    elif key == ord('w'): # Move zoom center up
        zoom_center_y = max(0.0, zoom_center_y - 0.05)
        print(f"Zoom center: ({zoom_center_x:.2f}, {zoom_center_y:.2f})")
    elif key == ord('s'): # Move zoom center down
        zoom_center_y = min(1.0, zoom_center_y + 0.05)
        print(f"Zoom center: ({zoom_center_x:.2f}, {zoom_center_y:.2f})")
    elif key == ord('a'): # Move zoom center left
        zoom_center_x = max(0.0, zoom_center_x - 0.05)
        print(f"Zoom center: ({zoom_center_x:.2f}, {zoom_center_y:.2f})")
    elif key == ord('d'): # Move zoom center right
        zoom_center_x = min(1.0, zoom_center_x + 0.05)
        print(f"Zoom center: ({zoom_center_x:.2f}, {zoom_center_y:.2f})")

cap.release()
cv2.destroyAllWindows()
print("Stream processing finished.")
