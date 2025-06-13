import cv2
from ultralytics import YOLO
import os
import time
from pymavlink import mavutil # Import pymavlink library

os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;udp|fflags;nobuffer|flags;low_delay|probesize;32|analyzeduration;0"

model = YOLO("../yolo11n.pt")

video_source = "rtsp://192.168.144.25:8554/main.264"

print("Starting stream processing...")

DRONE_CONNECTION_STRING = 'udp:192.168.144.25:14550'

master = None # Initialize master to None
try:
    print(f"Connecting to drone via MAVLink on {DRONE_CONNECTION_STRING}...")
    # Attempt to establish MAVLink connection
    master = mavutil.mavlink_connection(DRONE_CONNECTION_STRING)
    # Wait for the first heartbeat message from the drone to confirm connection
    master.wait_for_heartbeat()
    print("Heartbeat received. Drone connected successfully!")

    # It's good practice to get the system and component ID from the heartbeat
    # if you want to be explicit about targeting commands.
    # master.target_system = master.target_system
    # master.target_component = master.target_component

except Exception as e:
    print(f"ERROR: Could not connect to drone via MAVLink. Please check connection string and drone status. {e}")
    master = None # Ensure master is None if connection failed

def send_land_command(mav_connection):
    if not mav_connection:
        print("Cannot send LAND command: MAVLink connection not established.")
        return

    print("Sending LAND command to drone...")
    mav_connection.mav.command_long_send(
        mav_connection.target_system,
        mav_connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, # confirmation
        0, # param1: abort takeoff (0=false)
        0, # param2: empty
        0, # param3: empty
        0, # param4: empty
        0, # param5: latitude (ignored if 0)
        0, # param6: longitude (ignored if 0)
        0  # param7: altitude (ignored if 0)
    )
    print("LAND command sent!")

# --- Main Detection and Command Loop ---
# Run YOLO inference on the video stream.
results = model(video_source, stream=True, verbose=False)

# Flag to indicate if "LAND" command has been issued via MAVLink.
# This prevents sending the command multiple times for the same detection.
land_command_issued = False

try:
    # Iterate through each detection result (frame by frame)
    for result in results:
        # `result.plot()` draws bounding boxes and labels on the frame, returning an annotated image.
        annotated_frame = result.plot()

        # Print the raw bounding box information for debugging purposes.
        # This includes coordinates, confidence, and class IDs.
        # print(result.boxes) # Uncomment this if you want to see raw box data

        # --- Object Detection Logic for "Bat" ---
        # `result.boxes.cls` contains the detected class IDs for the current frame.
        # `result.names` is a dictionary mapping class IDs to human-readable class names.

        # Assume "Bat" is not detected by default in this frame
        bat_detected_in_frame = False

        # Iterate through the detected class IDs (integers).
        # It's important to check if result.boxes.cls is not empty before iterating.
        if result.boxes and result.boxes.cls is not None:
            for class_id_int in result.boxes.cls:
                # Convert the integer class ID from a tensor to a Python integer type.
                class_id = int(class_id_int)

                # Look up the class name using the `result.names` dictionary.
                detected_class_name = result.names.get(class_id, "Unknown") # Use .get() for safety

                # Check if the detected class name is "Bat" (case-sensitive)
                if detected_class_name == "Bat":
                    print(f"Detected: {detected_class_name} at {time.time()} - Confidence: {result.boxes.conf[0]:.2f}")
                    bat_detected_in_frame = True
                    break # Exit the inner loop once "Bat" is found in the current frame.

        if bat_detected_in_frame:
            if not land_command_issued:
                # If "Bat" is detected and the LAND command hasn't been issued yet:
                print("--- !!! BAT DETECTED - INITIATING LANDING SEQUENCE !!! ---")
                send_land_command(master) # Send the MAVLink LAND command
                land_command_issued = True # Set the flag to true to prevent future command sends.
        else:
            # If "Bat" is not detected in the current frame, and a LAND command was issued,
            # you might want to reset the flag if you intend for the drone to resume flight
            # if the bat disappears. For a single LAND command, this is not needed.
            # If you want to allow re-triggering, remove or modify the `land_command_issued` flag.
            pass


        # Display the annotated frame in a window.
        cv2.imshow("YOLO Real-time Detection", annotated_frame)

        # Wait for 1 millisecond for a key press.
        # If 'q' is pressed, break the loop to exit the stream processing.
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print(" 'q' pressed. Exiting video stream.")
            break

except KeyboardInterrupt:
    print("\nScript interrupted by user (Ctrl+C).")
except Exception as e:
    print(f"An error occurred during stream processing or MAVLink communication: {e}")
finally:
    # --- Cleanup ---
    # Release all OpenCV windows.
    cv2.destroyAllWindows()
    print("OpenCV windows closed.")

    # Close the MAVLink connection if it was established
    if master:
        print("Closing MAVLink connection.")
        master.close()
    print("Stream processing finished.")

