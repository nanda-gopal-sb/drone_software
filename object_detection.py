import cv2
from ultralytics import YOLO
import os
import time

# Set FFmpeg environment variables for low latency
# This needs to be set BEFORE cv2.VideoCapture is initialized
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp|fflags;nobuffer|flags;low_delay"

# Your RTSP URL
# Replace with your actual RTSP URL (e.g., rtsp://user:password@ip:port/stream)
RTSP_URL = "rtsp://192.168.144.25:8554/main.264"

# Load a pre-trained YOLO model (e.g., YOLOv8n for nano, can be yolov8s, yolov8m, etc.)
# You can also load your custom trained model: model = YOLO("path/to/your/best.pt")
model = YOLO("../yolo11n.pt")

def run_yolo_on_rtsp():
    print(f"Attempting to open RTSP stream: {RTSP_URL}")
    cap = cv2.VideoCapture(RTSP_URL, cv2.CAP_FFMPEG) # Explicitly use FFmpeg backend

    if not cap.isOpened():
        print(f"Error: Could not open RTSP stream {RTSP_URL}")
        print("Please check the URL, credentials, camera status, and network connectivity.")
        print("Also ensure OpenCV is compiled with FFmpeg support.")
        return

    print("RTSP stream opened successfully. Starting YOLO inference...")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame, stream ended or error occurred. Reconnecting...")
                cap.release()
                cap = cv2.VideoCapture(RTSP_URL, cv2.CAP_FFMPEG)
                if not cap.isOpened():
                    print("Error: Could not re-open RTSP stream. Exiting.")
                    break
                continue

            # Perform YOLO inference on the frame
            results = model(frame, stream=False, verbose=False) # stream=False for single frame inference, verbose=False to reduce console output

            # Process results (e.g., draw bounding boxes)
            annotated_frame = frame.copy()
            for r in results:
                boxes = r.boxes  # Boxes object for bounding box outputs
                for box in boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    conf = float(box.conf[0])
                    cls = int(box.cls[0])
                    label = model.names[cls]

                    # Draw bounding box
                    color = (0, 255, 0) # Green
                    cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(annotated_frame, f"{label} {conf:.2f}", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            # Display the annotated frame
            cv2.imshow("YOLO Real-time Detection", annotated_frame)

            # Break the loop if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except Exception as e:
        print(f"An error occurred during streaming or inference: {e}")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("Stream closed and windows destroyed.")

if __name__ == "__main__":
    run_yolo_on_rtsp()
