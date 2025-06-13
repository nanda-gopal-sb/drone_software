import cv2
from ultralytics import YOLO
import os
import time
from pymavlink import mavutil
import math # Import math for atan2

# --- YOLO Configuration ---
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;udp|fflags;nobuffer|flags;low_delay|probesize;32|analyzeduration;0"
model = YOLO("../yolo11n.pt")
source = "rtsp://192.168.144.25:8554/main.264" # Your RTSP stream source

# --- MAVLink Configuration ---
# Replace with your drone's connection string
# For SITL (Software In The Loop) simulation:
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
# For a real drone connected via UDP:
# master = mavutil.mavlink_connection('udp:<DRONE_IP>:<DRONE_PORT>')
# For a real drone connected via Serial:
# master = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600) # Adjust port and baud rate

# Wait for the first heartbeat from the drone
print("Waiting for drone heartbeat...")
master.wait_heartbeat()
print(f"Heartbeat from system (system {master.target_system} component {master.target_component})")

# --- Control Parameters ---
fixed_tracking_speed = 0.5 # Desired constant tracking speed in m/s (adjust this)
centering_threshold_pixels = 30 # How close (in pixels) to the center before stopping movement

# --- MAVLink Helper Function for sending velocity commands ---
def set_target_velocity(master, vx, vy, vz=0, yaw_rate=0):
    """
    Sends a SET_POSITION_TARGET_LOCAL_NED message to the drone to command velocities.
    vx: velocity in X (North) in m/s
    vy: velocity in Y (East) in m/s
    vz: velocity in Z (Down) in m/s - usually 0 for horizontal tracking
    yaw_rate: yaw rate in rad/s - usually 0 unless specific yaw control is needed
    """
    # Type_mask: Bitmask for ignoring specified position/velocity/acceleration components.
    # We want to command velocities in X, Y, Z, and yaw rate, so ignore position and acceleration.
    type_mask = (
        mavutil.mavlink.MAV_POS_TARGET_TYPE_MASK_X_IGNORE |
        mavutil.mavlink.MAV_POS_TARGET_TYPE_MASK_Y_IGNORE |
        mavutil.mavlink.MAV_POS_TARGET_TYPE_MASK_Z_IGNORE |
        mavutil.mavlink.MAV_POS_TARGET_TYPE_MASK_AX_IGNORE |
        mavutil.mavlink.MAV_POS_TARGET_TYPE_MASK_AY_IGNORE |
        mavutil.mavlink.MAV_POS_TARGET_TYPE_MASK_AZ_IGNORE |
        mavutil.mavlink.MAV_POS_TARGET_TYPE_MASK_YAW_IGNORE # We are not controlling yaw (but can send yaw_rate)
    )

    master.mav.set_position_target_local_ned_send(
        0,       # time_boot_ms (not used)
        master.target_system, # Target system
        master.target_component, # Target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # Frame
        0b0000111111000111, # type_mask (only speeds enabled: VX, VY, VZ)
        0, 0, 0, # x, y, z positions (not used)
        vx, vy, vz, # x, y, z velocities (m/s)
        0, 0, 0, # x, y, z accelerations (not used)
        0, yaw_rate) # yaw, yaw_rate (not used)

# --- Main Stream Processing Loop ---
print("Starting stream processing")

results = model(source, stream=True, verbose=False)

try:
    for result in results:
        annotated_frame = result.plot()
        boxes = result.boxes # Access the Boxes object

        frame_height, frame_width, _ = annotated_frame.shape
        frame_center_x = frame_width / 2
        frame_center_y = frame_height / 2

        target_box = None
        max_area = 0

        # Find the largest bounding box (our assumed target)
        if boxes:
            for box in boxes:
                # box.xywh gives [x_cent    er, y_center, width, height]
                x_center, y_center, width, height = box.xywh[0].cpu().numpy()
                area = width * height
                if area > max_area:
                    max_area = area
                    target_box = box

        if target_box is not None:
            # Get center coordinates of the target bounding box
            target_x_center, target_y_center, _, _ = target_box.xywh[0].cpu().numpy()

            # Calculate the direction vector (offset from frame center)
            offset_x = target_x_center - frame_center_x
            offset_y = target_y_center - frame_center_y

            # Calculate the magnitude of the offset (distance from center)
            offset_magnitude = math.sqrt(offset_x**2 + offset_y**2)

            # Print current offset for debugging
            print(f"Offset X: {offset_x:.2f} pixels, Offset Y: {offset_y:.2f} pixels, Magnitude: {offset_magnitude:.2f} pixels")

            # Check if object is centered
            if offset_magnitude < centering_threshold_pixels:
                print("Object centered. Stopping drone movement.")
                set_target_velocity(master, 0, 0, 0) # Stop all movement
            else:
                # Calculate the angle to the target
                # atan2(y, x) returns the angle in radians between the positive x-axis and the point (x, y).
                # In image coordinates:
                # - positive offset_x means target is to the right of center.
                # - positive offset_y means target is below center.
                #
                # Mapping to drone NED frame (North-East-Down):
                # - North (Vx): positive Y in image often means South (negative North).
                # - East (Vy): positive X in image often means East (positive East).
                #
                # So, if target is right (positive offset_x) -> move East (positive Vy)
                # If target is down (positive offset_y) -> move South (negative Vx)
                #
                # Therefore, the angle `theta` in the drone's horizontal plane
                # needs to be derived from these offsets.
                # A simple mapping for a downward-facing camera looking at a ground target:
                # angle = atan2(offset_x, -offset_y) # Angle relative to positive North (Vx) for Vy, Vx
                #
                # Let's use the offsets to get the direction:
                # The angle in the image plane from the perspective of the drone looking down.
                # The drone's X (North) velocity relates to vertical movement in the image.
                # The drone's Y (East) velocity relates to horizontal movement in the image.

                # These are typical mappings, but might need inversion based on your camera and drone setup.
                # If target is on screen RIGHT (positive offset_x), drone needs to move EAST (positive Vy)
                # If target is on screen DOWN (positive offset_y), drone needs to move SOUTH (negative Vx)

                # So, velocity_north (vx) corresponds to -offset_y
                # velocity_east (vy) corresponds to offset_x

                # Calculate the angle of the target relative to the frame center
                # atan2(y_component, x_component)
                # Here, we need to map image coordinates (offset_x, offset_y) to NED (North, East) for angle calculation.
                # If positive offset_x is towards East, and positive offset_y is towards South:
                angle_rad = math.atan2(offset_x, -offset_y) # Angle from North axis, clockwise

                # Calculate vx and vy based on fixed speed and angle
                # Vx (North velocity) = speed * cos(angle)
                # Vy (East velocity) = speed * sin(angle)
                drone_vx = fixed_tracking_speed * math.cos(angle_rad)
                drone_vy = fixed_tracking_speed * math.sin(angle_rad)

                print(f"Commanding drone velocity: Vx={drone_vx:.2f} m/s, Vy={drone_vy:.2f} m/s (Speed: {fixed_tracking_speed:.2f} m/s, Angle: {math.degrees(angle_rad):.2f} deg)")
                set_target_velocity(master, drone_vx, drone_vy, 0) # vz=0 for horizontal centering

        # Display the annotated frame
        cv2.imshow("YOLO Real-time Detection", annotated_frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("'q' pressed. Exiting.")
            break

except Exception as e:
    print(f"An error occurred: {e}")
finally:
    # Ensure drone stops on exit
    print("Stopping drone movement and releasing resources.")
    set_target_velocity(master, 0, 0, 0) # Send zero velocity to stop drone
    cv2.destroyAllWindows()
    master.close()
    print("Stream processing finished.")
