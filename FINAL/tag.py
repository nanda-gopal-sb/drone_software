import os
import cv2
import threading
import time
import subprocess
from pymavlink import mavutil
from ultralytics import YOLO

RTSP_URL = "rtsp://your_rtsp_stream"
YOLO_MODEL_PATH = "../../yolov8n.pt"
GPS_END = "38.31555,-76.55276"  # Leave empty for now
MAPPING_DIR = "MAPPING"
OBJECTS_FILE = "objects.txt"

os.makedirs(MAPPING_DIR, exist_ok=True)

gps_data = {"lat": 0, "lon": 0, "alt": 0}
gps_lock = threading.Lock()
stop_event = threading.Event()
class_dirs = set()
class_dirs_lock = threading.Lock()
class_threads_started = set()

model = YOLO(YOLO_MODEL_PATH)

def fetch_gps():
    mav = mavutil.mavlink_connection('udp:127.0.0.1:14550')
    while not stop_event.is_set():
        msg = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if msg:
            with gps_lock:
                gps_data['lat'] = msg.lat / 1e7
                gps_data['lon'] = msg.lon / 1e7
                gps_data['alt'] = msg.relative_alt / 1000.0
            if f"{gps_data['lat']},{gps_data['lon']}" == GPS_END:
                stop_event.set()

def write_exif(image_path, lat, lon, alt):
    subprocess.run([
        "exiftool",
        f"-GPSLatitude={lat}",
        f"-GPSLongitude={lon}",
        f"-GPSAltitude={alt}",
        "-overwrite_original",
        image_path
    ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def video_capture():
    cap = cv2.VideoCapture(0)
    count = 0
    last_time = time.time()
    while not stop_event.is_set() and cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            continue
        if time.time() - last_time >= 1:
            img_path = os.path.join(MAPPING_DIR, f"frame_{count}.jpg")
            with gps_lock:
                lat, lon, alt = gps_data['lat'], gps_data['lon'], gps_data['alt']
            cv2.imwrite(img_path, frame)
            write_exif(img_path, lat, lon, alt)
            last_time = time.time()
            count += 1
    cap.release()

def object_detection():
    while not stop_event.is_set():
        for fname in os.listdir(MAPPING_DIR):
            if not fname.endswith(".jpg"):
                continue
            fpath = os.path.join(MAPPING_DIR, fname)
            results = model(fpath)
            img = cv2.imread(fpath)
            h, w = img.shape[:2]

            for r in results:
                for box in r.boxes:
                    cls_id = int(box.cls)
                    label = model.names[cls_id]
                    with class_dirs_lock:
                        new_class = label not in class_dirs
                        class_dirs.add(label)
                        num_classes = len(class_dirs)

                    obj_dir = os.path.join(label)
                    os.makedirs(obj_dir, exist_ok=True)
                    xyxy = box.xyxy[0].cpu().numpy().astype(int)
                    cv2.rectangle(img, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), (0, 255, 0), 2)
                    cv2.imwrite(os.path.join(obj_dir, fname), img)

                    with class_dirs_lock:
                        if label not in class_threads_started and num_classes >= 2:
                            class_threads_started.add(label)
                            threading.Thread(target=process_class_dir, args=(label,)).start()

            os.remove(fpath)
        time.sleep(1)

def process_class_dir(label):
    dir_path = os.path.join(label)
    closest_img = None
    closest_dist = float("inf")
    closest_gps = (0, 0, 0)
    max_wait = 60  # wait max 60 seconds for frames to show up
    waited = 0

    while waited < max_wait:
        images = [f for f in os.listdir(dir_path) if f.endswith(".jpg")]
        if not images:
            time.sleep(1)
            waited += 1
            continue

        for img_file in images:
            img_path = os.path.join(dir_path, img_file)
            img = cv2.imread(img_path)
            if img is None:
                continue
            h, w = img.shape[:2]
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            _, thresh = cv2.threshold(gray, 10, 255, cv2.THRESH_BINARY)
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                x, y, bw, bh = cv2.boundingRect(cnt)
                box_cx = x + bw // 2
                box_cy = y + bh // 2
                frame_cx = w // 2
                frame_cy = h // 2
                dist = (box_cx - frame_cx)**2 + (box_cy - frame_cy)**2
                if dist < closest_dist:
                    closest_dist = dist
                    closest_img = img_file
                    with gps_lock:
                        closest_gps = (gps_data['lat'], gps_data['lon'], gps_data['alt'])

        if closest_img:
            break
        time.sleep(1)
        waited += 1

    if closest_img:
        with threading.Lock():  # protect file write
            with open(OBJECTS_FILE, "a") as f:
                f.write(f"{label}: {closest_gps[0]};{closest_gps[1]};{closest_gps[2]}\n")

threads = [
    threading.Thread(target=fetch_gps),
    threading.Thread(target=video_capture),
    threading.Thread(target=object_detection)
]

for t in threads:
    t.start()
for t in threads:
    t.join()
