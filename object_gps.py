import cv2
import time
import os
import subprocess
import threading
import queue
from pymavlink import mavutil
from ultralytics import YOLO

# --- Configuration ---
RTSP_STREAM_URL = "rtsp://192.168.144.25:8554/main.264"
MAVPROXY_PORT = "udp:127.0.0.1:14550"
OUTPUT_FRAME_DIR = "frames"
GPS_LOG_FILE = "gps_check.txt"
YOLO_MODEL_PATH = "../TRAIN/best.pt"
FRAME_PROCESS_INTERVAL = 1.0

os.makedirs(OUTPUT_FRAME_DIR, exist_ok=True)

frame_queue = queue.Queue(maxsize=12) # Queue for raw frames
gps_data_lock = threading.Lock()
current_gps_data = {
    "latitude": None,
    "longitude": None,
    "altitude": None,
    "timestamp": None
}
stop_event = threading.Event() # Event to signal threads to stop

master = None
try:
    master = mavutil.mavlink_connection(MAVPROXY_PORT)
    print(f"Attempting to connect to MAVProxy at {MAVPROXY_PORT}...")
except Exception as e:
    print(f"Error initializing MAVLink connection object: {e}")
    print("GPS data will not be available.")

model = None
try:
    model = YOLO(YOLO_MODEL_PATH)
    print(f"YOLO model loaded: {YOLO_MODEL_PATH}")
except Exception as e:
    print(f"Error loading YOLO model: {e}")
    stop_event.set() # Signal to stop if model fails to load

gps_log_file_handle = None
try:
    gps_log_file_handle = open(GPS_LOG_FILE, "a")
    print(f"GPS log file opened: {GPS_LOG_FILE}")
except Exception as e:
    print(f"Error opening GPS log file {GPS_LOG_FILE}: {e}")
    gps_log_file_handle = None


def write_gps_to_exif(image_path, latitude, longitude, altitude):
    if latitude is None or longitude is None:
        print(f"Warning: No valid GPS data to write to EXIF for {image_path}")
        return
    try:
        exiftool_command = [
            "exiftool",
            "-overwrite_original", 
            f"-GPSLatitude={latitude}",
            f"-GPSLongitude={longitude}",
            f"-GPSAltitude={altitude if altitude is not None else 0}",
            "-GPSAltitudeRef=0", # 0 for above sea level, 1 for below
            image_path
        ]
        
        result = subprocess.run(exiftool_command, capture_output=True, text=True, check=True)
        if "1 image files updated" in result.stdout:
            print(f"Successfully wrote GPS to EXIF for {image_path}")
        else:
            print(f"Failed to write GPS to EXIF for {image_path}: {result.stdout}")
            if result.stderr:
                print(f"ExifTool Error: {result.stderr}")
    except subprocess.CalledProcessError as e:
        print(f"Error calling ExifTool: {e}")
        print(f"Stdout: {e.stdout}")
        print(f"Stderr: {e.stderr}")
    except FileNotFoundError:
        print("Error: exiftool not found. Please install it and ensure it's in your PATH.")

def video_capture_thread():
    cap = cv2.VideoCapture(RTSP_STREAM_URL)
    if not cap.isOpened():
        print(f"Error: Could not open RTSP stream at {RTSP_STREAM_URL}")
        stop_event.set()
        return

    print("Video capture thread started.")
    last_frame_time = time.time()
    frame_counter = 0

    while not stop_event.is_set():
        ret, frame = cap.read()
        if not ret:
            print("Video capture thread: End of stream or error reading frame. Attempting to re-open...")
            cap.release()
            time.sleep(1)
            cap = cv2.VideoCapture(RTSP_STREAM_URL)
            if not cap.isOpened():
                print("Video capture thread: Failed to re-open stream. Exiting.")
                stop_event.set()
                break
            continue

        current_time = time.time()
        if current_time - last_frame_time >= FRAME_PROCESS_INTERVAL:
            try:
                frame_queue.put((frame.copy(), current_time), block=False)
                last_frame_time = current_time
                frame_counter += 1
            except queue.Full:
                pass

    print("Video capture thread stopping.")
    cap.release()

def gps_reader_thread():
    print("GPS reader thread started.")
    if not master:
        print("GPS reader thread: MAVLink connection not initialized. Exiting.")
        return

    master.wait_heartbeat()
    print("GPS reader thread: Heartbeat received from MAVProxy!")

    while not stop_event.is_set():
        try:
            msg = master.recv_match(type=['GPS_RAW_INT', 'GPS2_RAW'], blocking=False, timeout=0.1)
            if msg:
                if msg.get_type() in ['GPS_RAW_INT', 'GPS2_RAW']:
                    with gps_data_lock:
                        current_gps_data["latitude"] = msg.lat / 1e7
                        current_gps_data["longitude"] = msg.lon / 1e7
                        current_gps_data["altitude"] = msg.alt / 1e3 # Altitude in meters
                        current_gps_data["timestamp"] = time.time()
                    print(f"GPS reader thread: Lat={current_gps_data['latitude']:.6f}, Lon={current_gps_data['longitude']:.6f}")
        except Exception as e:
            print(f"GPS reader thread: Error reading MAVLink message: {e}")
        time.sleep(0.01) # Small sleep to prevent busy-waiting

    print("GPS reader thread stopping.")

def processing_thread():
    print("Processing thread started.")
    if not model:
        print("Processing thread: YOLO model not loaded. Exiting.")
        stop_event.set()
        return

    while not stop_event.is_set() or not frame_queue.empty():
        try:
            frame, frame_timestamp = frame_queue.get(timeout=1)
            print(f"\nProcessing thread: Got frame from queue at {time.ctime(frame_timestamp)}")

            # Get the current GPS data
            with gps_data_lock:
                frame_gps_data = current_gps_data.copy()

            # --- Save Frame ---
            frame_filename = os.path.join(OUTPUT_FRAME_DIR, f"frame_{int(frame_timestamp)}.jpg")
            cv2.imwrite(frame_filename, frame)
            print(f"Processing thread: Saved frame: {frame_filename}")

            # --- Write GPS to EXIF ---
            if frame_gps_data["latitude"] is not None and frame_gps_data["longitude"] is not None:
                write_gps_to_exif(
                    frame_filename,
                    frame_gps_data["latitude"],
                    frame_gps_data["longitude"],
                    frame_gps_data["altitude"]
                )
            else:
                print("Processing thread: Skipping EXIF write: GPS data not available.")

            # --- Run YOLO Inference ---
            print("Processing thread: Running YOLO inference...")
            results = model(frame, verbose=False)

            person_detected = False
            for r in results:
                for box in r.boxes:
                    class_id = int(box.cls)
                    if model.names[class_id] == "person":
                        person_detected = True
                        confidence = box.conf.item()
                        print(f"Processing thread: Person detected! Confidence: {confidence:.2f}")
                        break
                if person_detected:
                    break

            if person_detected:
                if gps_log_file_handle:
                    if frame_gps_data["latitude"] is not None and frame_gps_data["longitude"] is not None:
                        log_entry = (
                            f"Timestamp: {time.ctime(frame_timestamp)}, "
                            f"Latitude: {frame_gps_data['latitude']:.6f}, "
                            f"Longitude: {frame_gps_data['longitude']:.6f}, "
                            f"Altitude: {frame_gps_data['altitude'] if frame_gps_data['altitude'] is not None else 'N/A'}\n"
                        )
                        gps_log_file_handle.write(log_entry)
                        gps_log_file_handle.flush()
                        print(f"Processing thread: Wrote GPS location to {GPS_LOG_FILE}")
                    else:
                        print("Processing thread: Person detected, but no valid GPS data to log.")
                else:
                    print("Processing thread: Person detected, but GPS log file is not open.")

            frame_queue.task_done() # Mark the task as done for this item in the queue

        except queue.Empty:
            if stop_event.is_set():
                print("Processing thread: Queue empty and stop event set. Exiting.")
                break
            # print("Processing thread: Waiting for frames...")
            time.sleep(0.1) # Small sleep if queue is empty to avoid busy-waiting

    print("Processing thread stopping.")


# --- Main Execution ---
if __name__ == "__main__":
    threads = []

    # Start threads
    t_video = threading.Thread(target=video_capture_thread, daemon=True)
    threads.append(t_video)
    t_gps = threading.Thread(target=gps_reader_thread, daemon=True)
    threads.append(t_gps)
    t_processing = threading.Thread(target=processing_thread, daemon=True)
    threads.append(t_processing)

    for t in threads:
        t.start()

    print("Main thread: All threads started. Press Ctrl+C to stop.")

    try:
        while not stop_event.is_set():
            time.sleep(0.5) # Main thread just waits and keeps the program alive
    except KeyboardInterrupt:
        print("\nMain thread: Ctrl+C detected. Signaling threads to stop...")
    finally:
        stop_event.set() # Signal all threads to stop

        for t in threads:
            t.join() # Wait for all threads to complete

        if gps_log_file_handle:
            gps_log_file_handle.close()
        if master:
            master.close()
        print("Main thread: All threads stopped and resources released. Exiting.")
