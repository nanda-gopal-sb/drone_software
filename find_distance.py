import cv2
import math
from ultralytics import YOLO

def calculate_direction_to_biggest_bbox_center(image_width, image_height, bounding_boxes):
    if not bounding_boxes:
        print("No bounding boxes provided.")
        return None

    biggest_bbox = None
    max_area = 0

    for bbox in bounding_boxes:
        x_min, y_min, x_max, y_max = bbox['x_min'], bbox['y_min'], bbox['x_max'], bbox['y_max']
        width = x_max - x_min
        height = y_max - y_min
        area = width * height
        if area > max_area:
            max_area = area
            biggest_bbox = bbox

    if not biggest_bbox:
        print("Could not determine the biggest bounding box.")
        return None

    image_center_x = image_width / 2
    image_center_y = image_height / 2

    bbox_center_x = (biggest_bbox['x_min'] + biggest_bbox['x_max']) / 2
    bbox_center_y = (biggest_bbox['y_min'] + biggest_bbox['y_max']) / 2

    vector_x = bbox_center_x - image_center_x
    vector_y = image_center_y - bbox_center_y  # Y-axis is inverted for typical angles (up is positive)

    angle_rad = math.atan2(vector_y, vector_x)
    angle_deg = math.degrees(angle_rad)

    angle_deg = (angle_deg + 360) % 360

    return angle_deg

def run_yolov8_and_get_direction(image_path_or_array, model_path='../yolo11n.pt', confidence_threshold=0.5):
    try:
        model = YOLO(model_path)
    except Exception as e:
        print(f"Error loading YOLOv8 model: {e}")
        print("Please ensure you have 'ultralytics' installed (`pip install ultralytics`)")
        print(f"and that '{model_path}' is a valid YOLOv8 model file (it will download if not local).")
        return None, None

    if isinstance(image_path_or_array, str):
        image = cv2.imread(image_path_or_array)
        if image is None:
            print(f"Error: Could not load image from {image_path_or_array}")
            return None, None
    else:
        image = image_path_or_array

    image_height, image_width, _ = image.shape
    annotated_image = image.copy() # Create a copy to draw on

    results = model(image, conf=confidence_threshold)

    detected_bboxes = []
    if results and results[0].boxes: # Check if any results and bounding boxes are present
        for box in results[0].boxes:
            # box.xyxy returns x1, y1, x2, y2 coordinates
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            confidence = box.conf[0].item()

            detected_bboxes.append({
                'x_min': x1,
                'y_min': y1,
                'x_max': x2,
                'y_max': y2,
                'confidence': confidence
            })

            # Draw bounding box on the image
            cv2.rectangle(annotated_image, (x1, y1), (x2, y2), (0, 255, 0), 2) # Green box
            label = f"{model.names[int(box.cls[0].item())]} {confidence:.2f}"
            cv2.putText(annotated_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Calculate the direction
    direction_angle = calculate_direction_to_biggest_bbox_center(image_width, image_height, detected_bboxes)

    # Draw image center and direction line
    image_center_x = int(image_width / 2)
    image_center_y = int(image_height / 2)
    cv2.circle(annotated_image, (image_center_x, image_center_y), 5, (255, 0, 0), -1) # Blue dot for image center

    if direction_angle is not None:
        # Find the biggest bounding box again to get its center for drawing the line
        biggest_bbox = None
        max_area = 0
        for bbox in detected_bboxes:
            x_min, y_min, x_max, y_max = bbox['x_min'], bbox['y_min'], bbox['x_max'], bbox['y_max']
            area = (x_max - x_min) * (y_max - y_min)
            if area > max_area:
                max_area = area
                biggest_bbox = bbox

        if biggest_bbox:
            bbox_center_x = int((biggest_bbox['x_min'] + biggest_bbox['x_max']) / 2)
            bbox_center_y = int((biggest_bbox['y_min'] + biggest_bbox['y_max']) / 2)
            cv2.circle(annotated_image, (bbox_center_x, bbox_center_y), 5, (0, 0, 255), -1) # Red dot for bbox center
            cv2.line(annotated_image, (image_center_x, image_center_y), (bbox_center_x, bbox_center_y), (0, 255, 255), 2) # Yellow line
            cv2.putText(annotated_image, f"{direction_angle:.1f} deg",
                        (image_center_x + 10, image_center_y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    return direction_angle, annotated_image

# --- Main execution block ---

def calculate_direction_to_biggest_bbox_center(image_width, image_height, bounding_boxes):
    """
    Calculates the direction (angle) from the image center to the center
    of the biggest bounding box.

    Args:
        image_width (int): The width of the image.
        image_height (int): The height of the image.
        bounding_boxes (list of dict): A list of dictionaries, where each dictionary
                                       represents a bounding box with keys 'x_min',
                                       'y_min', 'x_max', 'y_max', and 'confidence'.
                                       (Note: YOLOv8 typically gives xywh or xyxy,
                                       we'll convert to x_min, y_min, x_max, y_max).

    Returns:
        float: The angle in degrees from the image center to the center
               of the biggest bounding box. 0 degrees is to the right,
               90 degrees is up, 180 degrees is to the left, and 270 degrees is down.
               Returns None if no bounding boxes are provided.
    """

    if not bounding_boxes:
        print("No bounding boxes provided.")
        return None

    # 1. Find the biggest bounding box
    biggest_bbox = None
    max_area = 0

    for bbox in bounding_boxes:
        x_min, y_min, x_max, y_max = bbox['x_min'], bbox['y_min'], bbox['x_max'], bbox['y_max']
        width = x_max - x_min
        height = y_max - y_min
        area = width * height
        if area > max_area:
            max_area = area
            biggest_bbox = bbox

    if not biggest_bbox:
        print("Could not determine the biggest bounding box.")
        return None

    # 2. Calculate image center
    image_center_x = image_width / 2
    image_center_y = image_height / 2

    # 3. Calculate the center of the biggest bounding box
    bbox_center_x = (biggest_bbox['x_min'] + biggest_bbox['x_max']) / 2
    bbox_center_y = (biggest_bbox['y_min'] + biggest_bbox['y_max']) / 2

    # 4. Calculate the vector from image center to bounding box center
    vector_x = bbox_center_x - image_center_x
    vector_y = image_center_y - bbox_center_y  # Y-axis is inverted for typical angles (up is positive)

    # 5. Calculate the angle
    angle_rad = math.atan2(vector_y, vector_x)
    angle_deg = math.degrees(angle_rad)

    # Normalize angle to be between 0 and 360 degrees
    angle_deg = (angle_deg + 360) % 360

    return angle_deg

def run_yolov8_and_get_direction(image_path_or_array, model_path='yolov11n.pt', confidence_threshold=0.5):
    """
    Runs YOLOv8 inference on an image and calculates the direction to the center
    of the biggest detected bounding box.

    Args:
        image_path_or_array (str or numpy.ndarray): Path to the image file or a numpy array representing the image.
        model_path (str): Path to the YOLOv8 model weights (e.g., 'yolov8n.pt' for nano, 'yolov8m.pt' for medium).
                          If not present locally, it will be downloaded.
        confidence_threshold (float): Minimum confidence score for a detection to be considered.

    Returns:
        tuple: A tuple containing (direction_angle, annotated_image).
               direction_angle is the angle in degrees, None if no objects detected.
               annotated_image is the image with detections and center/direction drawn.
    """
    try:
        # Load a YOLOv8 model
        model = YOLO(model_path)
    except Exception as e:
        print(f"Error loading YOLOv8 model: {e}")
        print("Please ensure you have 'ultralytics' installed (`pip install ultralytics`)")
        print(f"and that '{model_path}' is a valid YOLOv8 model file (it will download if not local).")
        return None, None

    # Load the image
    if isinstance(image_path_or_array, str):
        image = cv2.imread(image_path_or_array)
        if image is None:
            print(f"Error: Could not load image from {image_path_or_array}")
            return None, None
    else:
        image = image_path_or_array

    image_height, image_width, _ = image.shape
    annotated_image = image.copy() # Create a copy to draw on

    # Perform inference
    results = model(image, conf=confidence_threshold)

    detected_bboxes = []
    if results and results[0].boxes: # Check if any results and bounding boxes are present
        for box in results[0].boxes:
            # box.xyxy returns x1, y1, x2, y2 coordinates
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            confidence = box.conf[0].item()

            detected_bboxes.append({
                'x_min': x1,
                'y_min': y1,
                'x_max': x2,
                'y_max': y2,
                'confidence': confidence
            })

            # Draw bounding box on the image
            cv2.rectangle(annotated_image, (x1, y1), (x2, y2), (0, 255, 0), 2) # Green box
            label = f"{model.names[int(box.cls[0].item())]} {confidence:.2f}"
            cv2.putText(annotated_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Calculate the direction
    direction_angle = calculate_direction_to_biggest_bbox_center(image_width, image_height, detected_bboxes)

    # Draw image center and direction line
    image_center_x = int(image_width / 2)
    image_center_y = int(image_height / 2)
    cv2.circle(annotated_image, (image_center_x, image_center_y), 5, (255, 0, 0), -1) # Blue dot for image center

    if direction_angle is not None:
        # Find the biggest bounding box again to get its center for drawing the line
        biggest_bbox = None
        max_area = 0
        for bbox in detected_bboxes:
            x_min, y_min, x_max, y_max = bbox['x_min'], bbox['y_min'], bbox['x_max'], bbox['y_max']
            area = (x_max - x_min) * (y_max - y_min)
            if area > max_area:
                max_area = area
                biggest_bbox = bbox

        if biggest_bbox:
            bbox_center_x = int((biggest_bbox['x_min'] + biggest_bbox['x_max']) / 2)
            bbox_center_y = int((biggest_bbox['y_min'] + biggest_bbox['y_max']) / 2)
            cv2.circle(annotated_image, (bbox_center_x, bbox_center_y), 5, (0, 0, 255), -1) # Red dot for bbox center
            cv2.line(annotated_image, (image_center_x, image_center_y), (bbox_center_x, bbox_center_y), (0, 255, 255), 2) # Yellow line
            cv2.putText(annotated_image, f"{direction_angle:.1f} deg",
                        (image_center_x + 10, image_center_y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    return direction_angle, annotated_image

# --- Main execution block ---
if __name__ == "__main__":
    # Create a dummy image for demonstration if no actual image path is provided
    # This part is just for making the example runnable without external files

    image_to_process = "rtsp://192.168.144.25:8554/main.264"
    print("Using 'bus.jpg' for demonstration.")

    except Exception as e:
        print(f"Error preparing dummy image: {e}")
        print("Please ensure OpenCV is installed (`pip install opencv-python`)")
        image_to_process = None # Fallback if even dummy image creation fails

    if image_to_process:
        print(f"Running YOLOv8 on: {image_to_process}")
        direction, annotated_img = run_yolov8_and_get_direction(image_to_process, model_path='yolo11n.pt')

        if direction is not None:
            print(f"Direction to the center of the biggest object: {direction:.2f} degrees")
            if annotated_img is not None:
                cv2.imshow("YOLOv8 Detections with Direction", annotated_img)
                cv2.waitKey(0) # Wait indefinitely until a key is pressed
                cv2.destroyAllWindows()
        else:
            print("No objects detected or an error occurred.")
    else:
        print("Could not proceed without an image to process.")
