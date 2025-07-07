import subprocess
import numpy as np
import cv2
import time
from queue import Queue, Empty
from threading import Thread, Lock
from ultralytics import YOLO # Assuming you want to re-integrate YOLO

# --- Configuration ---
RTSP_URL = "rtsp://192.168.144.25:8554/main.264"  # Replace with your HM30 RTSP URL
YOLO_MODEL_PATH = "best.pt"
FRAME_WIDTH = 900
FRAME_HEIGHT = 900
FRAME_RATE = 30 # Output frame rate from FFmpeg
PIXEL_FORMAT = "bgr24" # Matching OpenCV's default BGR
BYTES_PER_FRAME = FRAME_WIDTH * FRAME_HEIGHT * 3 # 3 bytes per pixel for BGR24

CONFIDENCE_THRESHOLD = 0.75
DETECTION_PERSISTENCE_TIME = 3.0 # seconds

# Global dictionary to track object detection states
object_states = {}
object_states_lock = Lock()

# Queues for inter-thread communication
raw_frame_queue = Queue(maxsize=1) # Buffer for raw frames from FFmpeg
processed_frame_queue = Queue(maxsize=1) # Buffer for frames with detections (for display)

# --- FFmpeg Command ---
FFMPEG_COMMAND = [
    'ffmpeg',
    '-i', RTSP_URL,
    '-f', 'rawvideo',
    '-pix_fmt', PIXEL_FORMAT,
    '-s', f"{FRAME_WIDTH}x{FRAME_HEIGHT}",
    '-an', '-sn', # No audio, no subtitles
    '-r', str(FRAME_RATE), # Output frame rate
    # Low latency flags
    '-fflags', 'nobuffer',
    '-flags', 'low_delay',
    '-probesize', '32',
    '-analyzeduration', '0',
    'pipe:1' # Output to stdout
]

def ffmpeg_capture_thread():
    """Captures raw frames from FFmpeg subprocess and puts them into a queue."""
    print("FFmpeg capture thread started.")
    process = None

    while True:
        try:
            if process is None or process.poll() is not None: # Restart FFmpeg if not running or crashed
                print("Starting FFmpeg subprocess...")
                process = subprocess.Popen(
                    FFMPEG_COMMAND,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE # Capture stderr for debugging FFmpeg errors
                )
                print(f"FFmpeg PID: {process.pid}")
                # Read any initial FFmpeg output on stderr to clear buffer
                # This can sometimes help with faster startup
                time.sleep(0.5) # Give FFmpeg a moment to start

            raw_frame_data = process.stdout.read(BYTES_PER_FRAME)

            if not raw_frame_data:
                # This means FFmpeg probably exited or is not producing output
                print("FFmpeg did not produce expected data. Reconnecting...")
                process.terminate()
                process.wait() # Ensure process is truly dead
                time.sleep(1) # Wait before retrying
                continue

            if len(raw_frame_data) != BYTES_PER_FRAME:
                print(f"Warning: Incomplete frame received ({len(raw_frame_data)} bytes). Dropping.")
                continue # Drop incomplete frames

            # Convert raw bytes to NumPy array
            frame = np.frombuffer(raw_frame_data, np.uint8).reshape((FRAME_HEIGHT, FRAME_WIDTH, 3))

            # Put frame into the queue, prioritize latest frame
            try:
                while not raw_frame_queue.empty():
                    raw_frame_queue.get_nowait()
                raw_frame_queue.put(frame)
            except Exception as e:
                pass # Queue full, frame dropped

        except Exception as e:
            print(f"Error in FFmpeg capture thread: {e}")
            if process:
                process.terminate()
                process.wait()
            time.sleep(1) # Wait before attempting restart

def process_frames_thread():
    """Processes raw frames with YOLO and sends to display queue."""
    global object_states
    model = YOLO(YOLO_MODEL_PATH)
    print("YOLO inference thread started.")

    while True:
        try:
            frame = raw_frame_queue.get(timeout=1) # Get the latest raw frame
            current_time = time.time()
            frame_for_display = frame.copy() # Make a copy for drawing bounding boxes

            # Perform inference
            results = model.predict(frame, verbose=False, conf=CONFIDENCE_THRESHOLD)

            detected_objects_in_frame_this_cycle = set()

            with object_states_lock:
                for r in results:
                    for box in r.boxes:
                        confidence = float(box.conf[0])
                        class_id = int(box.cls[0])
                        object_name = model.names[class_id]

                        # Draw bounding boxes and labels for display
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        label = f"{object_name} {confidence:.2f}"
                        cv2.rectangle(frame_for_display, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(frame_for_display, label, (x1, y1 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                        if confidence >= CONFIDENCE_THRESHOLD:
                            detected_objects_in_frame_this_cycle.add(object_name)

                            # Update object state
                            if object_name not in object_states:
                                object_states[object_name] = {'first_detected_time': current_time,
                                                              'last_detected_time': current_time,
                                                              'sum_accuracy': confidence,
                                                              'count_detections': 1}
                            else:
                                state = object_states[object_name]
                                # If the object hasn't been seen for a while, reset its tracking
                                if (current_time - state['last_detected_time']) > DETECTION_PERSISTENCE_TIME:
                                    state['first_detected_time'] = current_time
                                    state['sum_accuracy'] = confidence
                                    state['count_detections'] = 1
                                else:
                                    state['sum_accuracy'] += confidence
                                    state['count_detections'] += 1
                                state['last_detected_time'] = current_time

                            state = object_states[object_name] # Get updated state reference

                            # Check if persistence criteria met and print to console
                            if (current_time - state['first_detected_time']) >= DETECTION_PERSISTENCE_TIME:
                                avg_accuracy = state['sum_accuracy'] / state['count_detections']
                                if avg_accuracy >= CONFIDENCE_THRESHOLD:
                                    print(f"[{object_name}] {avg_accuracy:.2f}%")
                                    # Reset for this object after reporting to avoid continuous spam
                                    object_states[object_name] = {'first_detected_time': current_time,
                                                                  'last_detected_time': current_time,
                                                                  'sum_accuracy': confidence,
                                                                  'count_detections': 1}

                # Clean up states for objects not detected in this frame
                keys_to_delete = [obj for obj in object_states if obj not in detected_objects_in_frame_this_cycle]
                for obj in keys_to_delete:
                    if (current_time - object_states[obj]['last_detected_time']) > DETECTION_PERSISTENCE_TIME * 1.5:
                        del object_states[obj]

            # Send the frame with bounding boxes to the display queue
            try:
                while not processed_frame_queue.empty():
                    processed_frame_queue.get_nowait()
                processed_frame_queue.put(frame_for_display)
            except Exception as e:
                pass # Queue full, frame dropped for display

        except Empty:
            pass # No frame in queue, continue looping
        except Exception as e:
            print(f"Error during frame processing: {e}")

def display_frames_thread():
    """Retrieves processed frames and displays them."""
    print("Display thread started.")
    cv2.namedWindow("HM30 RTSP Stream - YOLO Detections", cv2.WINDOW_NORMAL)

    while True:
        try:
            frame_to_display = processed_frame_queue.get(timeout=1)
            cv2.imshow("HM30 RTSP Stream - YOLO Detections", frame_to_display)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        except Empty:
            pass
        except Exception as e:
            print(f"Error during display: {e}")
            break

    cv2.destroyAllWindows()
    print("Display thread stopped.")


# --- Main Execution ---
if __name__ == "__main__":
    # Start the threads
    ffmpeg_thread = Thread(target=ffmpeg_capture_thread, daemon=True)
    yolo_thread = Thread(target=process_frames_thread, daemon=True)
    display_thread = Thread(target=display_frames_thread, daemon=True)

    ffmpeg_thread.start()
    yolo_thread.start()
    display_thread.start()

    print("Application started. Press 'q' in the display window or Ctrl+C to exit.")

    try:
        while True:
            time.sleep(1)
            if not display_thread.is_alive():
                print("Display window closed. Exiting application.")
                break
    except KeyboardInterrupt:
        print("KeyboardInterrupt detected. Exiting application.")
    finally:
        # Daemon threads will automatically terminate when the main thread exits
        pass
