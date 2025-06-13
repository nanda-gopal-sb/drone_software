import cv2
import os
from ultralytics import YOLO # Import YOLO for model inference

# --- 1. Set FFmpeg environment variables for low latency ---
# This needs to be set BEFORE cv2.VideoCapture is initialized.
# The format is key;value|key;value
# 'rtsp_transport;tcp' is generally recommended for stability over UDP
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp|fflags;nobuffer|flags;low_delay"

# --- 2. Define your RTSP URL ---
rtsp_url = "rtsp://192.168.144.25:8554/main.264"

# --- 3. Load a pre-trained YOLO model ---
# You can use a pre-trained model like yolov8n.pt, or your custom trained model.
# Make sure 'yolov8n.pt' (or your chosen model) is in the same directory,
# or provide the full path.
try:
    model = YOLO("../yolo11n.pt")
    print("YOLOv8n model loaded successfully.")
except Exception as e:
    print(f"Error loading YOLO model: {e}")
    print("Please ensure 'yolov8n.pt' is available or use a valid model path.")
    exit() # Exit if model cannot be loaded

# --- 4. Initialize VideoCapture with FFmpeg backend ---
# Explicitly using cv2.CAP_FFMPEG ensures the FFmpeg backend is used,
# which will pick up the environment variables.
print(f"Attempting to open RTSP stream: {rtsp_url} with low-latency flags...")
cap = cv2.VideoCapture(rtsp_url, cv2.CAP_FFMPEG)

if not cap.isOpened():
    print(f"Error: Could not open RTSP stream at {rtsp_url}")
    print("Please check the URL, camera status, network connectivity, and ensure OpenCV is compiled with FFmpeg support.")
    # Check if the environment variable was set
    print(f"OPENCV_FFMPEG_CAPTURE_OPTIONS: {os.getenv('OPENCV_FFMPEG_CAPTURE_OPTIONS')}")
    exit()

print("RTSP stream opened successfully. Starting YOLO inference...")

# --- 5. Main loop for reading frames and running inference ---
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame. Stream might have ended or connection lost. Attempting to reconnect...")
            cap.release() # Release existing capture
            cap = cv2.VideoCapture(rtsp_url, cv2.CAP_FFMPEG) # Try to re-open
            if not cap.isOpened():
                print("Error: Could not re-open RTSP stream. Exiting.")
                break
            continue # Skip to the next iteration to read the new frame

        # Perform YOLO inference on the frame
        # stream=False is for single-frame inference, verbose=False reduces console output
        results = model(frame, stream=False, verbose=False)

        # Process results and draw annotations on the frame
        annotated_frame = frame.copy() # Make a copy to draw on
        for r in results:
            boxes = r.boxes  # Bounding box outputs
            # Iterate through detected boxes
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0]) # Get box coordinates
                conf = float(box.conf[0])             # Confidence score
                cls = int(box.cls[0])                 # Class ID
                label = model.names[cls]              # Class name

                # Draw bounding box and label
                color = (0, 255, 0) # Green color for boxes
                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(annotated_frame, f"{label} {conf:.2f}", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            # If you also have masks, keypoints, etc., you can plot them:
            # annotated_frame = r.plot() # This single line plots everything (boxes, masks, etc.)
                                      # but if you only need specific elements or custom drawing,
                                      # iterating 'r.boxes' as above gives more control.


        # Display the annotated frame
        cv2.imshow("YOLO Real-time Detection", annotated_frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Program interrupted by user.")
except Exception as e:
    print(f"An unexpected error occurred: {e}")
finally:
    # --- 6. Release resources ---
    cap.release()
    cv2.destroyAllWindows()
    print("Stream closed and windows destroyed.")
