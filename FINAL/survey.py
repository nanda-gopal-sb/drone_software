import math
import time
from pymavlink import mavutil, mavwp

def haversine_distance(lat1, lon1, lat2, lon2):
    R = 6371000  # Radius of Earth in meters
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    a = math.sin(delta_phi / 2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

# --- Main Drone Survey Function ---

def survey_rectangular_field(coordinates, altitude, line_spacing, connection_string, target_system=1, target_component=1):
    print(f"Connecting to MAVLink: {connection_string}...")
    try:
        master = mavutil.mavlink_connection(connection_string, baud=115200)
    except Exception as e:
        print(f"Error connecting to MAVLink: {e}")
        print("Please ensure the connection string is correct and the flight controller is running.")
        return

    # Wait for the first heartbeat to ensure connection
    print("Waiting for heartbeat...")
    try:
        master.wait_heartbeat()
        print(f"Heartbeat from system (SYSID {master.target_system} COMPID {master.target_component})")
    except Exception as e:
        print(f"No heartbeat received: {e}")
        print("Ensure the flight controller is powered on and connected.")
        return

    # Update target system and component from the received heartbeat
    target_system = master.target_system
    target_component = master.target_component
    print(f"Using drone SYSID: {target_system}, COMPID: {target_component}")

    # --- Mode and Arming ---
    print("Setting mode to GUIDED...")
    # Mode 4 is GUIDED for ArduPilot. If using PX4, refer to its mode numbers.
    # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED is used for custom modes like GUIDED.
    master.mav.command_long_send(
        target_system,
        target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,  # Confirmation
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        4,  # Custom mode value (4 for GUIDED in ArduPilot)
        0, 0, 0, 0, 0 # Parameters 3-7 (unused)
    )
    # Give the flight controller a moment to process the mode change
    time.sleep(1)

    print("Arming motors (if not already armed)...")
    master.mav.command_long_send(
        target_system,
        target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # Confirmation
        1,  # 1 to arm, 0 to disarm
        0, 0, 0, 0, 0, 0 # Parameters 2-7 (unused)
    )
    # Wait until motors are armed. This is a blocking call.
    try:
        master.motors_armed_wait()
        print("Motors armed.")
    except Exception as e:
        print(f"Failed to arm motors: {e}")
        print("Please ensure the drone is safe to arm (e.g., pre-arm checks passed).")
        return
    TAKEOFF_ALTITUDE = 16.0
    TAKEOFF_PITCH = 0.0 # Degrees. Use 0 for vertical takeoff.
    print(f"Sending TAKEOFF command to {TAKEOFF_ALTITUDE} meters with pitch {TAKEOFF_PITCH}...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, # confirmation
        TAKEOFF_PITCH, # param1: pitch
        0, 0, 0, 0, 0, # empty params
        TAKEOFF_ALTITUDE # param7: altitude
    )
    print(f"Waiting for drone to reach {TAKEOFF_ALTITUDE}m altitude...")
    while True:
        msg = master.recv_match(type=['GLOBAL_POSITION_INT'], blocking=True, timeout=1)
        if msg:
            current_alt_rel = msg.relative_alt / 1000.0 # GLOBAL_POSITION_INT.relative_alt is in mm
            print(f"Current relative altitude: {current_alt_rel:.2f}m / {TAKEOFF_ALTITUDE:.2f}m")
            if current_alt_rel >= TAKEOFF_ALTITUDE - 1:
                print(f"Drone reached target takeoff altitude of {current_alt_rel:.2f}m.")
                break
        time.sleep(0.5) # Check more frequently during takeoff

    # --- Waypoint Generation ---
    print("Generating survey waypoints...")
    # Find the bounding box of the given coordinates
    min_lat = min(c[0] for c in coordinates)
    max_lat = max(c[0] for c in coordinates)
    min_lon = min(c[1] for c in coordinates)
    max_lon = max(c[1] for c in coordinates)

    # Calculate approximate degrees per meter at the average latitude of the field.
    # This is an approximation for small areas. For larger areas or high precision,
    # a projection to a local East-North-Up (ENU) frame would be more robust.
    avg_lat = (min_lat + max_lat) / 2
    # Rough conversion factors (vary slightly with latitude):
    # ~111,320 meters per degree of latitude
    # ~111,320 * cos(latitude) meters per degree of longitude
    meters_per_deg_lat = 111320.0
    meters_per_deg_lon = 111320.0 * math.cos(math.radians(avg_lat))

    # Convert line spacing from meters to degrees of latitude
    line_spacing_deg_lat = line_spacing / meters_per_deg_lat

    waypoints = []
    current_lat = min_lat
    # Start sweeping East (from min_lon to max_lon)
    sweep_direction_east = True

    # Generate parallel lines, alternating sweep direction (zig-zag pattern)
    # Add a small margin to max_lat to ensure the last line is included
    while current_lat <= max_lat + line_spacing_deg_lat * 0.5:
        if sweep_direction_east:
            # Fly from left to right (west to east)
            waypoints.append((current_lat, min_lon))
            waypoints.append((current_lat, max_lon))
        else:
            # Fly from right to left (east to west)
            waypoints.append((current_lat, max_lon))
            waypoints.append((current_lat, min_lon))

        current_lat += line_spacing_deg_lat
        sweep_direction_east = not sweep_direction_east

    print(f"Generated {len(waypoints)} waypoints.")
    for i, wp in enumerate(waypoints):
        print(f"Waypoint {i+1:3}: Lat={wp[0]:.7f}, Lon={wp[1]:.7f}")

    # --- Execute Waypoints ---
    print("\nExecuting survey path...")
    accuracy_threshold_meters = 2.0  # Consider waypoint reached if within this distance
    altitude_accuracy_threshold_meters = 1.0 # Consider altitude reached if within this difference
    waypoint_timeout_seconds = 120   # Max time to reach a waypoint

    for i, (target_lat, target_lon) in enumerate(waypoints):
        print(f"\nNavigating to Waypoint {i+1}/{len(waypoints)}: "
              f"({target_lat:.6f}, {target_lon:.6f}) at altitude {altitude}m")
        master.mav.set_position_target_global_int_send(
            0,  # time_boot_ms (not used with type_mask)
            target_system,
            target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, # Frame: Global Lat/Lon/Alt relative to home
            0b110111111000, # Only use position, ignore velocity, accel, yaw,
            int(target_lat * 1e7),  # Latitude in degrees * 1e7
            int(target_lon * 1e7),  # Longitude in degrees * 1e7
            altitude,               # Altitude in meters (relative to home)
            0, 0, 0,                # Vx, Vy, Vz (ignored by type_mask)
            0, 0, 0,                # AFx, AFy, AFz (ignored by type_mask)
            0, 0                    # Yaw, Yaw Rate (ignored by type_mask)
        )
        waypoint_reached = False
        start_time = time.time()

        while not waypoint_reached and (time.time() - start_time) < waypoint_timeout_seconds:
            # Receive messages from the flight controller
            msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=0.1)
            if msg:
                # GLOBAL_POSITION_INT provides current GPS coordinates and altitude
                current_lat_drone = msg.lat / 1e7
                current_lon_drone = msg.lon / 1e7
                current_alt_drone = msg.relative_alt / 1000.0 # Altitude is in millimeters in this message

                dist_to_waypoint = haversine_distance(current_lat_drone, current_lon_drone, target_lat, target_lon)
                alt_difference = abs(current_alt_drone - altitude)

                print(f"  Drone Pos: ({current_lat_drone:.6f}, {current_lon_drone:.6f}) "
                      f"Alt: {current_alt_drone:.1f}m, "
                      f"Dist to WP: {dist_to_waypoint:.1f}m, "
                      f"Alt Diff: {alt_difference:.1f}m")

                # Check if the drone is within the accuracy threshold of the waypoint
                if dist_to_waypoint < accuracy_threshold_meters and \
                   alt_difference < altitude_accuracy_threshold_meters:
                    waypoint_reached = True
                    print(f"Waypoint {i+1} reached!")
                else:
                    time.sleep(0.5) # Wait a bit before next position check
            else:
                time.sleep(0.1) # No message received, try again soon

        if not waypoint_reached:
            print(f"Warning: Drone did not reach Waypoint {i+1} within {waypoint_timeout_seconds} seconds. "
                  "This could be due to connectivity issues, flight controller problems, or external factors.")
            # In a real-world application, you might want to implement error recovery,
            # such as attempting to re-send the command, initiating RTL, or landing.
            # For this example, we will proceed to the next waypoint.
            # If a critical waypoint is missed, consider adding a `return` or `sys.exit()` here.

    print("\nSurvey path completed. Initiating Return To Launch (RTL) mode.")
    # Set mode to RTL (mode 6 for ArduPilot).
    master.mav.command_long_send(
        target_system,
        target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        6,  # Custom mode value (6 for RTL in ArduPilot)
        0, 0, 0, 0, 0
    )
    print("Drone should now be returning to launch. Monitor its progress manually or via GCS.")

    # It's good practice to close the connection when done
    master.close()
    print("MAVLink connection closed.")

# --- Example Usage ---
if __name__ == "__main__":
    # Define the coordinates of the rectangular field
    # (latitude, longitude) tuples
    field_coordinates = [
        (38.315386, -76.550875),  # Corner 1
        (38.315683, -76.552586),  # Corner 2
        (38.315895, -76.552519),  # Corner 3
        (38.315607, -76.550800)   # Corner 4
    ]
    survey_altitude_meters = 15 
    survey_line_spacing_meters = 10 

    mavlink_connection_string = 'udp:127.0.0.1:14550' # Default for SITL

    print("--- Starting Drone Field Survey Simulation ---")
    print(f"Field Coordinates: {field_coordinates}")
    print(f"Survey Altitude: {survey_altitude_meters} meters")
    print(f"Line Spacing: {survey_line_spacing_meters} meters")
    print(f"MAVLink Connection: {mavlink_connection_string}")
    print("\nIMPORTANT: This script is for demonstration. Always test in a simulator first.")
    print("Ensure your drone is armed and ready for flight before running on hardware.")
    print("Monitor the drone's behavior closely and be ready to take manual control if needed.")

    survey_rectangular_field(
        field_coordinates,
        survey_altitude_meters,
        survey_line_spacing_meters,
        mavlink_connection_string
    )
    print("\n--- Drone Field Survey Simulation Finished ---")
