from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink
import time
import math

# --- Connection Setup ---
# Replace 'udp:127.0.0.1:14550' with your drone's actual connection string
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')

# Wait for the first heartbeat to establish connection
master.wait_heartbeat()
print(f"Connected to system: {master.target_system}, component: {master.target_component}")

# --- Helper Functions ---

def set_mode(master, mode_name):
    """Sets the drone's flight mode."""
    mode_id = master.mode_mapping().get(mode_name)
    if mode_id is None:
        print(f"Mode '{mode_name}' not found in mode mapping.")
        return False

    print(f"Attempting to set mode to {mode_name} (custom mode {mode_id})...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0, # Confirmation (0 for no confirmation)
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id, # Custom mode number
        0, 0, 0, 0, 0 # Unused parameters
    )

    # Wait for the mode change to be acknowledged
    start_time = time.time()
    while True:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=0.1)
        if msg and mavutil.mode_string_v10(msg) == mode_name:
            print(f"Mode successfully set to {mode_name}!")
            return True
        elif time.time() - start_time > 5:
            print(f"Failed to set mode to {mode_name} or mode change not confirmed.")
            return False
        time.sleep(0.1)

def arm_drone(master):
    """Arms the drone."""
    print("Attempting to arm drone...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, # Confirmation
        1, # param1: 1 for arm, 0 for disarm
        0, 0, 0, 0, 0, 0 # Unused parameters
    )
    start_time = time.time()
    return True
    # while True:
    #     msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=0.1)
    #     if msg and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_ARMED):
    #         print("Drone armed!")
    #         return True
    #     elif time.time() - start_time > 10: # Timeout after 10 seconds
    #         print("Failed to arm drone or arming not confirmed.")
    #         return False
    #     time.sleep(0.1)

def takeoff(master, target_altitude):
    """Initiates takeoff to a specified altitude."""
    print(f"Attempting to takeoff to {target_altitude} meters...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, # Confirmation
        0, 0, 0, 0, # Unused parameters
        0, 0, target_altitude # lat, lon (ignored), altitude
    )

    start_time = time.time()
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=0.1)
        if msg:
            current_alt = msg.relative_alt / 1000.0 # relative_alt is in mm
            print(f"Current altitude: {current_alt:.2f}m")
            if current_alt >= target_altitude * 0.95: # Within 95% of target
                print("Reached takeoff altitude!")
                return True
        elif time.time() - start_time > 30: # Timeout after 30 seconds
            print("Failed to reach takeoff altitude.")
            return False
        time.sleep(0.1)

def goto_waypoint(master, lat, lon, alt, velocity_mps=5):
    """
    Sends a SET_POSITION_TARGET_GLOBAL_INT message to move the drone to a specific
    global latitude, longitude, and altitude in GUIDED mode.
    """
    print(f"Sending waypoint: Lat={lat}, Lon={lon}, Alt={alt}m, Speed={velocity_mps}m/s")

    # Type mask for position only (lat, lon, alt are active, others ignored)
    # type_mask_position_only = (
    #     mavlink.MAV_POS_TARGET_TYPE_IGNORE_VX |
    #     mavlink.MAV_POS_TARGET_TYPE_IGNORE_VY |
    #     mavlink.MAV_POS_TARGET_TYPE_IGNORE_VZ |
    #     mavlink.MAV_POS_TARGET_TYPE_IGNORE_AX |
    #     mavlink.MAV_POS_TARGET_TYPE_IGNORE_AY |
    #     mavlink.MAV_POS_TARGET_TYPE_IGNORE_AZ |
    #     mavlink.MAV_POS_TARGET_TYPE_IGNORE_YAW |
    #     mavlink.MAV_POS_TARGET_TYPE_IGNORE_YAW_RATE
    # )

    # Convert lat/lon to int32_t (degrees * 1e7)
    lat_int = int(lat * 1e7)
    lon_int = int(lon * 1e7)
    alt_m = alt # Altitude in meters

    master.mav.set_position_target_global_int_send(
        0, # time_boot_ms (not used here)
        master.target_system,
        master.target_component,
        mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, # Frame: Altitude relative to home
        0b0000111111000111,
        lat_int, # Latitude
        lon_int, # Longitude
        alt_m,   # Altitude
        velocity_mps, # vx (North velocity in m/s) - this field is often used as a desired groundspeed
        0, # vy (East velocity in m/s)
        0, # vz (Down velocity in m/s)
        0, 0, 0, # acceleration (not used)
        0, # yaw (not used)
        0  # yaw_rate (not used)
    )

def get_distance_meters(lat1, lon1, lat2, lon2):
    """
    Calculates the great-circle distance between two points on Earth (Haversine formula).
    Returns distance in meters.
    """
    R = 6371000  # Radius of Earth in meters
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)

    dlon = lon2_rad - lon1_rad
    dlat = lat2_rad - lat1_rad

    a = math.sin(dlat / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c
    return distance

def wait_for_waypoint(master, target_lat, target_lon, target_alt, acceptance_radius=5, alt_tolerance=1):
    """
    Waits until the drone is within the acceptance radius of the target waypoint.
    """
    print(f"Waiting for drone to reach Lat: {target_lat}, Lon: {target_lon}, Alt: {target_alt}m (radius: {acceptance_radius}m)...")
    start_time = time.time()
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=0.1)
        if msg:
            current_lat = msg.lat / 1e7
            current_lon = msg.lon / 1e7
            current_alt = msg.relative_alt / 1000.0 # relative_alt is in mm

            horizontal_distance = get_distance_meters(current_lat, current_lon, target_lat, target_lon)
            vertical_distance = abs(current_alt - target_alt)

            print(f"Current Pos: Lat {current_lat:.6f}, Lon {current_lon:.6f}, Alt {current_alt:.2f}m | "
                  f"H_Dist: {horizontal_distance:.2f}m, V_Dist: {vertical_distance:.2f}m")

            if horizontal_distance < acceptance_radius and vertical_distance < alt_tolerance:
                print("Waypoint reached!")
                return True
        elif time.time() - start_time > 200: # Timeout after 60 seconds for a waypoint
            print("Waypoint not reached within timeout period. Moving to next or stopping.")
            return False
        time.sleep(0.1)

# --- Main Script ---

# 1. Set Mode to GUIDED
if not set_mode(master, 'GUIDED'):
    print("Failed to set mode. Exiting.")
    exit()

# 2. Arm Drone
if not arm_drone(master):
    print("Failed to arm drone. Exiting.")
    exit()

# 3. Takeoff
takeoff_altitude = 20 # meters
if not takeoff(master, takeoff_altitude):
    print("Failed to takeoff. Exiting.")
    exit()

print("\nDrone is airborne and ready for mission.")
time.sleep(2) # Give it a moment after takeoff

# --- Define Waypoints for Circular Path ---
# These are example waypoints. Adjust for your specific location and desired path.
# (Latitude, Longitude, Altitude in meters AGL, Velocity in m/s)
# Ensure altitudes are consistent with your takeoff altitude or desired flight path.
waypoints = [
    (35.361991,149.165979, 20, 5)
]

num_repetitions = 3 # How many times to repeat the 1->2->3->1 cycle

current_waypoint_index = 0
current_repetition = 0

print(f"\nStarting circular mission for {num_repetitions} repetitions...")

try:
    while current_repetition < num_repetitions:
        target_lat, target_lon, target_alt, target_vel = waypoints[current_waypoint_index]

        # Send the waypoint command
        goto_waypoint(master, target_lat, target_lon, target_alt, target_vel)

        # Wait for the drone to reach the waypoint
        if not wait_for_waypoint(master, target_lat, target_lon, target_alt):
            print(f"Drone failed to reach Waypoint {current_waypoint_index + 1}. Aborting mission.")
            break # Exit the loop if a waypoint isn't reached

        # Move to the next waypoint in the sequence
        current_waypoint_index += 1

        # Check if we've completed a full circle (1->2->3)
        if current_waypoint_index >= len(waypoints):
            print(f"\nCompleted Circle {current_repetition + 1}!")
            current_waypoint_index = 0 # Loop back to the first waypoint
            current_repetition += 1

        time.sleep(1) # Small delay before sending next command

except KeyboardInterrupt:
    print("\nMission interrupted by user.")
finally:
    print("Mission completed or interrupted. Landing drone...")
    # Land the drone
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, # Confirmation
        0, 0, 0, 0, # Unused parameters
        0, 0, 0 # lat, lon, alt (ignored for LAND)
    )
    print("Sent LAND command.")
    # Optional: Disarm after landing
    # time.sleep(15) # Wait for landing
    # master.mav.command_long_send(master.target_system, master.target_component,
    #                              mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
    # print("Sent DISARM command.")
