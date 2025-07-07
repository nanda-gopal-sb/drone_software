import cv2
import math
from ultralytics import YOLO
import os

# Set OpenCV FFMPEG capture options for low latency
# These are crucial for minimizing delay in RTSP streams.
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;udp|fflags;nobuffer|flags;low_delay|probesize;32|analyzeduration;0|sync;skip_frames"

model = YOLO("yolo11n.pt")

source = "rtsp://192.168.144.25:8554/main.264"

print("Starting stream processing...")

def calculate_direction_and_centers(image_width, image_height, bounding_boxes_data):
    if not bounding_boxes_data:
        return None, None, None, None, None

    biggest_bbox = None
    max_area = 0

    for bbox in bounding_boxes_data:
        x_min, y_min, x_max, y_max = bbox['x_min'], bbox['y_min'], bbox['x_max'], bbox['y_max']
        width = x_max - x_min
        height = y_max - y_min
        area = width * height
        if area > max_area:
            max_area = area
            biggest_bbox = bbox

    if not biggest_bbox: # Should not happen if bounding_boxes_data is not empty, but good for safety
        return None, None, None, None, None

    # 2. Calculate image center
    image_center_x = image_width / 2
    image_center_y = image_height / 2

    # 3. Calculate the center of the biggest bounding box
    bbox_center_x = (biggest_bbox['x_min'] + biggest_bbox['x_max']) / 2
    bbox_center_y = (biggest_bbox['y_min'] + biggest_bbox['y_max']) / 2

    # 4. Calculate the vector from image center to bounding box center
    vector_x = bbox_center_x - image_center_x
    vector_y = image_center_y - bbox_center_y  # Y-axis is inverted for typical angles (up is positive)

    # 5. Calculate the angle using atan2 for correct quadrant handling
    angle_rad = math.atan2(vector_y, vector_x)
    angle_deg = math.degrees(angle_rad)

    # Normalize angle to be between 0 and 360 degrees
    angle_deg = (angle_deg + 360) % 360

    return angle_deg, int(image_center_x), int(image_center_y), int(bbox_center_x), int(bbox_center_y)


# The YOLO model's stream processing will now use the configured FFMPEG options
results = model(source, stream=True, verbose=False)

for result in results:
    # Get the original frame (numpy array) from the result for custom drawing
    frame = result.orig_img
    if frame is None:
        print("Warning: Received empty frame. Skipping.")
        continue # Skip if frame is not available

    image_height, image_width, _ = frame.shape

    # Let YOLO's built-in plot function draw the bounding boxes and labels first
    annotated_frame = result.plot()

    detected_bboxes = []
    # Check if any bounding boxes are present in the current result
    if result.boxes:
        for box in result.boxes:
            # Extract coordinates (x_min, y_min, x_max, y_max) and convert to int
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            confidence = box.conf[0].item() # Get confidence score

            detected_bboxes.append({
                'x_min': x1,
                'y_min': y1,
                'x_max': x2,
                'y_max': y2,
                'confidence': confidence
            })

    # Calculate direction and all necessary center coordinates
    angle, img_cx, img_cy, bbox_cx, bbox_cy = calculate_direction_and_centers(
        image_width, image_height, detected_bboxes
    )

    # If an object was detected and its direction calculated, draw the elements
    if angle is not None:
        # Draw image center as a blue dot
        cv2.circle(annotated_frame, (img_cx, img_cy), 5, (255, 0, 0), -1)

        # Draw the center of the biggest bounding box as a red dot
        cv2.circle(annotated_frame, (bbox_cx, bbox_cy), 5, (0, 0, 255), -1)

        # Draw a yellow line from the image center to the bounding box center
        cv2.line(annotated_frame, (img_cx, img_cy), (bbox_cx, bbox_cy), (0, 255, 255), 2)

        # Put the angle text near the image center
        cv2.putText(annotated_frame, f"{angle:.1f} deg",
                    (img_cx + 10, img_cy - 10), # Offset slightly for readability
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2) # Yellow text

        # Optional: You can also print the angle to console for debugging
        # print(f"Angle to biggest object: {angle:.1f} degrees")

    # Display the processed frame with YOLO detections and custom drawings
    cv2.imshow("YOLO Real-time Detection with Direction", annotated_frame)

    # Check for 'q' key press to quit the stream
    if cv2.waitKey(1) & 0xFF == ord('q'):
        print(" 'q' pressed. Exiting stream.")
        break

# Clean up: close all OpenCV windows
cv2.destroyAllWindows()
print("Stream processing finished.")
