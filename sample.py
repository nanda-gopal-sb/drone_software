from pymavlink import mavutil
import time
def set_target_velocity(master, vx, vy, vz=0, yaw_rate=0):
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

CONNECTION_STRING = 'udp:127.0.0.1:14550'
TARGET_SYSTEM = 1  # Often 1 for the main autopilot
TARGET_COMPONENT = 1 # Often 1 for the main autopilot (MAV_COMP_ID_AUTOPILOT1)

# --- Connect to the drone ---
print(f"Connecting to MAVLink: {CONNECTION_STRING}...")
try:
    master = mavutil.mavlink_connection(CONNECTION_STRING)
except Exception as e:
    print(f"Error connecting: {e}")
    exit()

# Wait for the first heartbeat to establish connection
print("Waiting for heartbeat...")
master.wait_heartbeat()
print(f"Heartbeat received from system (system {master.target_system}, component {master.target_component})")

# Update target system and component based on the heartbeat
TARGET_SYSTEM = master.target_system
TARGET_COMPONENT = master.target_component

# --- Arm the drone ---
print("Attempting to arm the drone...")
# Set up a listener for HEARTBEAT messages to check arming status
start_time = time.time()
while time.time() - start_time < 20: # Wait up to 20 seconds for arming
    # MAV_CMD_COMPONENT_ARM_DISARM is the standard way to arm/disarm
    master.mav.command_long_send(
        TARGET_SYSTEM,
        TARGET_COMPONENT,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # Confirmation (0 for no confirmation)
        1,  # 1 to arm, 0 to disarm
        0, 0, 0, 0, 0, 0
    )
    msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
    if msg:
        if msg.base_mode:
            print("Drone armed successfully!")
            break
    else:
        print("Waiting for heartbeat to confirm arming...")
    time.sleep(1) # Wait a bit before retrying arm command
else:
    print("Failed to arm the drone within the timeout. Check pre-arm checks and connection.")
    # Exit if arming failed, as subsequent steps depend on it
    exit()

# --- Set mode to GUIDED ---
print("Attempting to set mode to GUIDED...")
# Get the MAVLink mode ID for 'GUIDED'
# This requires knowing the specific autopilot type (e.g., ArduCopter, ArduPlane)
# and its mode definitions. For ArduCopter, GUIDED is typically mode 4.
# It's safer to get the mode ID from the master.mode_mapping dictionary if available.

# Try to find GUIDED mode in the autopilot's reported modes
guided_mode_id = None
if hasattr(master, 'mode_mapping'):
    for mode_name, mode_id in master.mode_mapping().items():
        if mode_name == 'GUIDED':
            guided_mode_id = mode_id
            break

if guided_mode_id is None:
    print("Could not find 'GUIDED' mode ID from autopilot's mode mapping. Assuming ArduCopter GUIDED mode (4).")
    # Fallback for ArduCopter if mode_mapping isn't robustly populated yet
    guided_mode_id = 4 # Common ID for GUIDED mode in ArduCopter

# Set mode using MAV_CMD_DO_SET_MODE
master.mav.command_long_send(
    TARGET_SYSTEM,
    TARGET_COMPONENT,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    0, # Confirmation
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, # Use this flag for custom modes (like GUIDED)
    guided_mode_id, # The custom mode ID (e.g., 4 for GUIDED in ArduCopter)
    0, 0, 0, 0, 0 # Unused parameters
)

# Wait for a HEARTBEAT message to confirm the mode change
start_time = time.time()
while time.time() - start_time < 10: # Wait up to 10 seconds for mode change
    msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
    if msg:
        current_base_mode = msg.base_mode
        current_custom_mode = msg.custom_mode

        # Check if the desired custom mode is active
        if current_base_mode & mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED and \
           current_custom_mode == guided_mode_id:
            print("Drone successfully set to GUIDED mode!")
            break
        else:
            print(f"Current mode: {master.flightmode}, custom_mode: {current_custom_mode}. Waiting for GUIDED.")
    else:
        print("Waiting for heartbeat to confirm mode change...")
    time.sleep(0.5)
else:
    print("Failed to set mode to GUIDED within the timeout.")




master.mav.set_position_target_local_ned_send(
        0,       # time_boot_ms (not used)
        master.target_system, # Target system
        master.target_component, # Target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # Frame
        0b0000111111000111, # type_mask (only speeds enabled: VX, VY, VZ)
        0, 0, 0, # x, y, z positions (not used)
        10, 20, 30, # x, y, z velocities (m/s)
        0, 0, 0, # x, y, z accelerations (not used)
        0, 0) 
print("Script finished.")

# You might want to add a disarm command here or further actions in GUIDED mode
# master.mav.command_long_send(TARGET_SYSTEM, TARGET_COMPONENT, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)