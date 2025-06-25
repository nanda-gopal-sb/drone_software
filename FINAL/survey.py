# Import necessary libraries
import math
import time
from pymavlink import mavutil

# --- Helper function for Haversine distance (needed for waypoint checking) ---
def haversine_distance(lat1, lon1, lat2, lon2):
    """
    Calculates the Haversine distance between two points on Earth
    given their latitudes and longitudes.
    Returns distance in meters.
    """
    R = 6371000  # Radius of Earth in meters (approximate)
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)

    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad

    a = math.sin(dlat / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    distance = R * c
    return distance

# --- Mock Variables (replace with actual values in a real setup) ---
# Example MAVLink connection string for SITL (Software In The Loop) simulation.
# For a real drone, this might be 'udp:0.0.0.0:14550' for a UDP connection,
# or a serial port like '/dev/ttyUSB0' with a specific baud rate.
connection_string = 'udp:127.0.0.1:14550'

# Example airdrop coordinates defining the rectangular boundary of your survey area.
# In a real scenario, these would be the actual corners of your field.
airdrop_coordinates = [
        (38.315386, -76.550875),  # Corner 1
        (38.315683, -76.552586),  # Corner 2
        (38.315895, -76.552519),  # Corner 3
        (38.315607, -76.550800)   # Corner 4
]

def survey_rectangular_field(target_system=1, target_component=1):
    """
    Performs a rectangular field survey mission with a drone.
    The drone flies in a zigzag pattern, stopping at each waypoint to allow for video feed.

    Args:
        target_system (int): MAVLink system ID of the drone. Default to 1.
        target_component (int): MAVLink component ID of the drone. Default to 1.
    """
    altitude = 18 # Target altitude for the survey in meters (relative to home).
    
    # MODIFICATION 1: Set the line spacing to 7.9 meters. This defines the distance
    # between the parallel lines of the zigzag survey pattern.
    line_spacing = 7.9 # meters between parallel survey lines (grid points)

    print(f"Connecting to MAVLink: {connection_string}...")
    try:
        # Establish MAVLink connection to the flight controller.
        master = mavutil.mavlink_connection(connection_string, baud=115200)
    except Exception as e:
        print(f"Error connecting to MAVLink: {e}")
        print("Please ensure the connection string is correct and the flight controller is running.")
        return

    # Wait for the first heartbeat message from the drone to confirm connection.
    print("Waiting for heartbeat...")
    try:
        master.wait_heartbeat()
        print(f"Heartbeat from system (SYSID {master.target_system} COMPID {master.target_component})")
    except Exception as e:
        print(f"No heartbeat received: {e}")
        print("Ensure the flight controller is powered on and connected.")
        return

    # Update target system and component IDs from the received heartbeat.
    # This ensures we are communicating with the correct drone.
    target_system = master.target_system
    target_component = master.target_component
    print(f"Using drone SYSID: {target_system}, COMPID: {target_component}")

    # --- Drone Mode and Arming ---
    print("Setting mode to GUIDED...")
    # Send a MAVLink command to set the drone's mode to GUIDED.
    # GUIDED mode allows the drone to be controlled by MAVLink position commands.
    master.mav.command_long_send(
        target_system,
        target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,  # Confirmation byte
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, # Flag to enable custom mode
        4,  # Custom mode value (4 typically corresponds to GUIDED mode in ArduPilot)
        0, 0, 0, 0, 0 # Unused parameters
    )
    time.sleep(1) # Give the flight controller a moment to process the mode change.

    print("Arming motors (if not already armed)...")
    # Send a MAVLink command to arm the drone's motors.
    master.mav.command_long_send(
        target_system,
        target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # Confirmation byte
        1,  # Parameter 1: 1 to arm, 0 to disarm
        0, 0, 0, 0, 0, 0 # Unused parameters
    )
    # Wait until the motors are confirmed armed. This is a blocking call.
    try:
        master.motors_armed_wait()
        print("Motors armed.")
    except Exception as e:
        print(f"Failed to arm motors: {e}")
        print("Please ensure the drone is safe to arm (e.g., pre-arm checks passed and GPS lock).")
        return

    # MODIFICATION 2: Set a slow ground speed for the drone during the mission.
    # This command tells the flight controller to maintain a specific ground speed
    # when navigating between waypoints.
    slow_ground_speed = 0.8 # Target ground speed in meters/second
    print(f"Setting ground speed to {slow_ground_speed} m/s...")
    master.mav.command_long_send(
        target_system,
        target_component,
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
        0, # Confirmation byte
        1, # Speed type: 1 means Groundspeed
        slow_ground_speed, # The desired speed in m/s
        -1, # Throttle (unused for groundspeed type, set to -1)
        0, 0, 0, 0 # Unused parameters
    )
    time.sleep(1) # Allow the drone to register the new speed setting.

    # --- Waypoint Generation for Survey Path ---
    print("Generating survey waypoints...")
    # Determine the bounding box of the survey area from the provided coordinates.
    min_lat = min(c[0] for c in airdrop_coordinates)
    max_lat = max(c[0] for c in airdrop_coordinates)
    min_lon = min(c[1] for c in airdrop_coordinates)
    max_lon = max(c[1] for c in airdrop_coordinates)

    # Calculate approximate degrees per meter at the average latitude.
    # This is a simplification for small areas; for high precision over large areas,
    # a more sophisticated geodetic projection might be needed.
    avg_lat = (min_lat + max_lat) / 2
    meters_per_deg_lat = 111320.0 # Approximate meters per degree of latitude

    # Convert the desired line spacing from meters to degrees of latitude.
    line_spacing_deg_lat = line_spacing / meters_per_deg_lat

    waypoints = [] # List to store the generated (latitude, longitude) waypoints
    current_lat = min_lat
    sweep_direction_east = True # Flag to alternate the sweep direction (zig-zag)

    # Generate parallel lines, sweeping across the longitude range.
    # The loop continues until the 'current_lat' exceeds the 'max_lat',
    # with a small buffer to ensure the last line is included.
    while current_lat <= max_lat + line_spacing_deg_lat * 0.5:
        if sweep_direction_east:
            # If sweeping east, add waypoints from min_lon to max_lon at current_lat.
            waypoints.append((current_lat, min_lon))
            waypoints.append((current_lat, max_lon))
        else:
            # If sweeping west, add waypoints from max_lon to min_lon at current_lat.
            waypoints.append((current_lat, max_lon))
            waypoints.append((current_lat, min_lon))

        current_lat += line_spacing_deg_lat # Move to the next survey line
        sweep_direction_east = not sweep_direction_east # Reverse direction for the next line

    print(f"Generated {len(waypoints)} waypoints.")
    for i, wp in enumerate(waypoints):
        print(f"Waypoint {i+1:3}: Lat={wp[0]:.7f}, Lon={wp[1]:.7f}")

    # --- Execute Waypoints ---
    print("\nExecuting survey path...")
    accuracy_threshold_meters = 2.0  # Distance (in meters) to consider a waypoint reached.
    altitude_accuracy_threshold_meters = 1.0 # Altitude difference (in meters) to consider target altitude reached.
    waypoint_timeout_seconds = 120   # Maximum time allowed to reach a single waypoint.
    # MODIFICATION 3: Set the dwell time at each waypoint to 3 seconds.
    dwell_time_at_waypoint = 3 # seconds the drone will pause at each waypoint.

    for i, (target_lat, target_lon) in enumerate(waypoints):
        print(f"\nNavigating to Waypoint {i+1}/{len(waypoints)}: "
              f"({target_lat:.6f}, {target_lon:.6f}) at altitude {altitude}m")
        
        # Send a SET_POSITION_TARGET_GLOBAL_INT MAVLink message.
        # This command tells the drone to go to a specific global latitude, longitude, and altitude.
        # The type_mask (0b110111111000) indicates that only position (lat, lon, alt)
        # should be used, while velocity, acceleration, and yaw components are ignored.
        # The drone's speed will be governed by the earlier DO_CHANGE_SPEED command.
        master.mav.set_position_target_global_int_send(
            0,  # time_boot_ms (not used when type_mask is active)
            target_system,
            target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, # Frame: Global coordinates relative to home altitude
            0b110111111000, # Type mask: only position is used (lat, lon, alt). Velocities, accel, yaw ignored.
            int(target_lat * 1e7),  # Latitude in degrees * 10^7
            int(target_lon * 1e7),  # Longitude in degrees * 10^7
            altitude,               # Altitude in meters (relative to home)
            0, 0, 0,                # Vx, Vy, Vz (ignored by type_mask)
            0, 0, 0,                # AFx, AFy, AFz (ignored by type_mask)
            0, 0                    # Yaw, Yaw Rate (ignored by type_mask)
        )
        
        waypoint_reached = False
        start_time = time.time()

        # Loop to monitor the drone's position until the waypoint is reached or timeout occurs.
        while not waypoint_reached and (time.time() - start_time) < waypoint_timeout_seconds:
            # Receive GLOBAL_POSITION_INT messages, which provide the drone's current GPS position and altitude.
            msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=0.1)
            if msg:
                current_lat_drone = msg.lat / 1e7      # Drone's current latitude (degrees)
                current_lon_drone = msg.lon / 1e7      # Drone's current longitude (degrees)
                current_alt_drone = msg.relative_alt / 1000.0 # Drone's current relative altitude (meters, from millimeters)

                # Calculate the 2D horizontal distance to the target waypoint.
                dist_to_waypoint = haversine_distance(current_lat_drone, current_lon_drone, target_lat, target_lon)
                # Calculate the absolute difference in altitude.
                alt_difference = abs(current_alt_drone - altitude)

                print(f"  Drone Pos: ({current_lat_drone:.6f}, {current_lon_drone:.6f}) "
                      f"Alt: {current_alt_drone:.1f}m, "
                      f"Dist to WP: {dist_to_waypoint:.1f}m, "
                      f"Alt Diff: {alt_difference:.1f}m")

                # Check if the drone is within the defined accuracy thresholds for position and altitude.
                if dist_to_waypoint < accuracy_threshold_meters and \
                   alt_difference < altitude_accuracy_threshold_meters:
                    waypoint_reached = True
                    print(f"Waypoint {i+1} reached!")
                    
                    # MODIFICATION 3: Pause the script for the specified dwell time at the waypoint.
                    print(f"Pausing for {dwell_time_at_waypoint} seconds at waypoint...")
                    time.sleep(dwell_time_at_waypoint)
                else:
                    time.sleep(0.5) # Wait before checking position again to avoid busy-waiting.
            else:
                time.sleep(0.1) # No message received, wait a short time and try again.
        
        # If the waypoint was not reached within the timeout period, print a warning.
        if not waypoint_reached:
            print(f"Warning: Drone did not reach Waypoint {i+1} within {waypoint_timeout_seconds} seconds. "
                  "This could be due to connectivity issues, flight controller problems, or external factors.")
            # Fallback behavior: attempt to send the drone to a fixed pre-defined safety point.
            # Correcting original syntax error: `int(38.315386, * 1e7)` to `int(38.315386 * 1e7)`
            master.mav.set_position_target_global_int_send(
                0,  
                target_system,
                target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, # Frame: Global Lat/Lon/Alt relative to home
                0b110111111000, # Only use position, ignore velocity, accel, yaw,
                int(38.315386 * 1e7),  # Fallback Latitude in degrees * 10^7
                int(-76.542861 * 1e7),  # Fallback Longitude in degrees * 10^7 (added a longitude for clarity)
                16,               # Fallback Altitude in meters (relative to home)
                0, 0, 0,                # Vx, Vy, Vz (ignored by type_mask)
                0, 0, 0,                # AFx, AFy, AFz (ignored by type_mask)
                0, 0                    # Yaw, Yaw Rate (ignored by type_mask)
            )

# Example usage (uncomment and replace with your actual values to run):
if __name__ == "__main__":
    # Make sure connection_string and airdrop_coordinates are set correctly
    # For a real drone, you might need to adjust connection_string.
    # For SITL, start `sim_vehicle.py --map --console` first.
    survey_rectangular_field()
