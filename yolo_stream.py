from ultralytics import YOLO
import cv2 # Still useful for display if not using model.show()

# Your RTSP URL
RTSP_URL = "rtsp://192.168.144.25:8554/main.264"

# Load a pre-trained YOLO model
model = YOLO("../yolov11n.pt")

def run_yolo_ultralytics_stream():
    print(f"Attempting to run YOLO on RTSP stream: {RTSP_URL}")
    try:
        # model.predict can take an RTSP URL directly and handle streaming
        # Ultralytics aims for real-time performance, so it likely incorporates low-latency
        # strategies internally. You might not need to set explicit FFmpeg flags here,
        # but the environment variable approach (Method 1) is still a good fallback
        # if you observe latency.
        results_generator = model.predict(source=RTSP_URL, stream=True, show=True, verbose=False)

        print("YOLO inference started on stream. Press 'q' to quit (if 'show=True' creates a window).")

        for r in results_generator:
            # If show=True, Ultralytics handles displaying the annotated frame.
            # If you want more control or custom drawing, you would do it here:
            # annotated_frame = r.plot() # Get the annotated frame as a numpy array
            # cv2.imshow("YOLO Stream", annotated_frame)
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     break
            pass # No explicit imshow needed if show=True

    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    run_yolo_ultralytics_stream()
