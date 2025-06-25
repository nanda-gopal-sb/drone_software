import time
import math
from pymavlink import mavutil
import sys # Import sys for exiting the script

# --- Constants for waypoint navigation ---
WAYPOINT_REACH_THRESHOLD_M = 3.0  # Horizontal distance in meters to consider waypoint reached
ALTITUDE_REACH_THRESHOLD_M = 1.0  # Vertical distance in meters to consider target altitude reached
connection_string = 'udp:127.0.0.1:14550' # You can change this if needed
airdrop_coordinates = [
        (38.315386, -76.550875),  # Corner 1
        (38.315683, -76.552586),  # Corner 2
        (38.315895, -76.552519),  # Corner 3
        (38.315607, -76.550800)   # Corner 4
    ]
survey_altitude_meters = 15 
survey_line_spacing_meters = 7 
try:
        master = mavutil.mavlink_connection(connection_string, baud=57600)
        master.wait_heartbeat()
        print(f"Heartbeat from system (sys {master.target_system} comp {master.target_component})")
except Exception as e:
        print(f"Error connecting to drone: {e}")
        print("Please ensure your drone is connected and the connection string is correct.")
        sys.exit(1) # Exit if connection fails

def haversine_distance(lat1, lon1, lat2, lon2):
    """
    Calculate the distance between two points on Earth using the Haversine formula.
    Returns distance in meters.
    """
    R = 6371000  # Radius of Earth in meters
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    a = math.sin(delta_phi / 2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c
    return distance

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
    slow_ground_speed = 3 # Target ground speed in meters/second
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

def read_waypoints_from_file(file_path):
    """
    Reads waypoints from a specified file.
    File format: latitude;longitude;altitude (one waypoint per line).
    Returns a list of dictionaries, or None if an error occurs.
    """
    waypoints = []
    try:
        with open(file_path, 'r') as f:
            for line_num, line in enumerate(f, 1):
                line = line.strip()
                if not line: # Skip empty lines
                    continue
                parts = line.split(';')
                if len(parts) != 3:
                    print(f"Error: Line {line_num} in '{file_path}' has incorrect format. Expected 'lat;lon;alt'. Got: '{line}'")
                    return None
                try:
                    lat = float(parts[0])
                    lon = float(parts[1])
                    alt = float(parts[2])
                    if alt < 0:
                        print(f"Warning: Line {line_num} in '{file_path}' has a negative altitude. Adjusting to 0 for safety.")
                        alt = 0.0
                    waypoints.append({'lat': lat, 'lon': lon, 'alt': alt})
                except ValueError:
                    print(f"Error: Line {line_num} in '{file_path}' contains non-numeric values. Got: '{line}'")
                    return None
    except FileNotFoundError:
        print(f"Error: Waypoint file not found at '{file_path}'")
        return None
    except Exception as e:
        print(f"An unexpected error occurred while reading the waypoint file: {e}")
        return None
    
    if not waypoints:
        print("Warning: No waypoints found in the file. Please check the file content.")
        return None

    return waypoints

def dropPayload(location,pin):
    master.mav.set_position_target_global_int_send(
        0, # time_boot_ms (not used, FC sets)
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, # Frame for target (relative to takeoff alt)
        0b110111111000, # Only use position, ignore velocity, accel, yaw
        int(location['lat'] * 1e7), # lat_int (degrees * 10^7)
        int(location['lon'] * 1e7), # lon_int (degrees * 10^7)
        location['alt'], # alt (meters)
        0, 0, 0, # vx, vy, vz (ignored by mask)
        0, 0, 0, # afx, afy, afz (ignored by mask)
        0, 0 # yaw, yaw_rate (ignored by mask)
    )
    while True:
        msg = master.recv_match(type=['GLOBAL_POSITION_INT'], blocking=True, timeout=1)
        if msg:
            current_lat = msg.lat / 1e7 
            current_lon = msg.lon / 1e7
            current_alt_rel = msg.relative_alt / 1000.0 
            dist_to_waypoint = haversine_distance(current_lat, current_lon, location['lat'], location['lon'])
            alt_diff = abs(current_alt_rel - location['alt'])
            print(f"Current Pos: Lat={current_lat:.6f}, Lon={current_lon:.6f}, Alt={current_alt_rel:.2f}m | Dist to WP: {dist_to_waypoint:.2f}m | Alt Diff: {alt_diff:.2f}m")
            if dist_to_waypoint <= WAYPOINT_REACH_THRESHOLD_M and alt_diff <= ALTITUDE_REACH_THRESHOLD_M:
                print(f"Reached waypoint")
                break
        time.sleep(0.1)
    control_servo(pin)

def control_servo(servo_pin: int, high_pwm: int = 2000, low_pwm: int = 1100, delay_seconds: int = 15):
    master_connection_string = "udp:127.0.0.1:14450"
    print(f"Attempting to connect to MAVLink autopilot at: {master_connection_string}")
    master.wait_heartbeat()
    print(f"Heartbeat from system (system {master.target_system} component {master.target_component})")

    # --- 3. Send the command ---
    print("Sending MAV_CMD_DO_SET_SERVO command...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,      # confirmation
        9,      # param1: servo instance number (channel 9)
        0,   # param2: PWM value
        0, 0, 0, 0, 0 # unused params
    )
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,      # confirmation
        10,      # param1: servo instance number (channel 9)
        0,   # param2: PWM value
        0, 0, 0, 0, 0 # unused params
    )
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,      # confirmation
        11,      # param1: servo instance number (channel 9)
        0,   # param2: PWM value
        0, 0, 0, 0, 0 # unused params
    )
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,      # confirmation
        12,      # param1: servo instance number (channel 9)
        0,   # param2: PWM value
        0, 0, 0, 0, 0 # unused params
    )
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,      # confirmation
        servo_pin,      # param1: servo instance number (channel 9)
        2000,   # param2: PWM value
        0, 0, 0, 0, 0 # unused params
    )
    print("Command sent.")
    time.sleep(15)
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,      # confirmation
        servo_pin,      # param1: servo instance number (channel 9)
        0,   # param2: PWM value
        0, 0, 0, 0, 0 # unused params
    )
    if 'master' in locals() and master.target_system is not None:
        master.close()
        print("MAVLink connection closed.")

def run_drone_mission():
    """
    Sets up and executes a drone mission using waypoints from a file,
    including heartbeat, arm, takeoff, and real-time waypoint navigation.
    """
    
    # --- Configuration: Waypoint File Path and Connection String ---
    # Waypoint file is now assumed to be 'waypoints.txt' in the same directory.
    waypoint_file_path = "waypoints.txt"
    # Common SITL connection string
    connection_string = 'udp:127.0.0.1:14550' # You can change this if needed

    waypoints = read_waypoints_from_file(waypoint_file_path)
    if waypoints is None:
        print("Failed to load waypoints. Exiting mission.")
        sys.exit(1) # Exit the script if waypoints couldn't be loaded
    
    num_waypoints = len(waypoints)

    # --- Display configured waypoints and ask for confirmation ---
    print("\n--- Loaded Waypoints ---")
    for i, wp in enumerate(waypoints):
        print(f"Waypoint {i+1}: Lat={wp['lat']:.6f}, Lon={wp['lon']:.6f}, Alt={wp['alt']:.2f}m")
    print(f"\nConnection String: {connection_string}")
    
    # NUM_CIRCLES moved into run_drone_mission scope as it's directly used here
    NUM_CIRCLES = 4 
    print(f"Total {num_waypoints} waypoints loaded. Mission will cover these {NUM_CIRCLES} times.")

    while True:
        confirmation = input("Do you want to start the drone mission? (yes/no): ").lower().strip()
        if confirmation == 'yes':
            print("Starting mission...")
            break
        elif confirmation == 'no':
            print("Mission aborted by user. Exiting.")
            sys.exit(0) # Exit cleanly
        else:
            print("Invalid input. Please type 'yes' or 'no'.")

    print("Setting drone mode to GUIDED...")
    mode_id = master.mode_mapping()['GUIDED']
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    # Simplified mode change wait as per no time checks. Still blocks.
    while True:
        msg = master.recv_match(type=['HEARTBEAT'], blocking=True, timeout=1)
        if msg:
            current_mode = mavutil.mode_string_v10(msg)
            if current_mode == "GUIDED":
                print("Drone is in GUIDED mode.")
                break
            else:
                print(f"Current mode: {current_mode}. Waiting for GUIDED mode...")
        time.sleep(0.5)

    # --- 5. Send ARM command ---
    print("Sending ARM command...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # confirmation
        1,  # arm
        0, 0, 0, 0, 0, 0
    )
    print("Waiting for drone to arm...")
    master.motors_armed_wait()
    print("Drone armed!")
    time.sleep(2)

    # --- 6. Send the liftoff command to 16m ---
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

    # Wait until the drone reaches takeoff altitude - removed timeout
    print(f"Waiting for drone to reach {TAKEOFF_ALTITUDE}m altitude...")
    while True:
        msg = master.recv_match(type=['GLOBAL_POSITION_INT'], blocking=True, timeout=1)
        if msg:
            current_alt_rel = msg.relative_alt / 1000.0 # GLOBAL_POSITION_INT.relative_alt is in mm
            print(f"Current relative altitude: {current_alt_rel:.2f}m / {TAKEOFF_ALTITUDE:.2f}m")
            if current_alt_rel >= TAKEOFF_ALTITUDE - ALTITUDE_REACH_THRESHOLD_M:
                print(f"Drone reached target takeoff altitude of {current_alt_rel:.2f}m.")
                break
        time.sleep(0.5) # Check more frequently during takeoff
    NUM_CIRCLES = 4
    for circle in range(NUM_CIRCLES):
        print(f"\n--- Starting Mission Circle {circle + 1}/{NUM_CIRCLES} ---")
        for i, wp in enumerate(waypoints):
            print(f"Navigating to waypoint {i + 1} (Circle {circle + 1}): Lat={wp['lat']:.6f}, Lon={wp['lon']:.6f}, Alt={wp['alt']:.2f}m")

            # Send SET_POSITION_TARGET_GLOBAL_INT command
            master.mav.set_position_target_global_int_send(
                0, # time_boot_ms (not used, FC sets)
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, # Frame for target (relative to takeoff alt)
                0b110111111000, # Only use position, ignore velocity, accel, yaw
                int(wp['lat'] * 1e7), # lat_int (degrees * 10^7)
                int(wp['lon'] * 1e7), # lon_int (degrees * 10^7)
                wp['alt'], # alt (meters)
                0, 0, 0, # vx, vy, vz (ignored by mask)
                0, 0, 0, # afx, afy, afz (ignored by mask)
                0, 0 # yaw, yaw_rate (ignored by mask)
            )

            # Wait for the drone to reach the waypoint by checking its GPS position - removed timeout
            while True:
                msg = master.recv_match(type=['GLOBAL_POSITION_INT'], blocking=True, timeout=1)
                if msg:
                    current_lat = msg.lat / 1e7  # GLOBAL_POSITION_INT.lat/lon are in degrees * 10^7
                    current_lon = msg.lon / 1e7
                    current_alt_rel = msg.relative_alt / 1000.0 # GLOBAL_POSITION_INT.relative_alt is in mm

                    dist_to_waypoint = haversine_distance(current_lat, current_lon, wp['lat'], wp['lon'])
                    alt_diff = abs(current_alt_rel - wp['alt'])

                    print(f"Current Pos: Lat={current_lat:.6f}, Lon={current_lon:.6f}, Alt={current_alt_rel:.2f}m | Dist to WP: {dist_to_waypoint:.2f}m | Alt Diff: {alt_diff:.2f}m")

                    if dist_to_waypoint <= WAYPOINT_REACH_THRESHOLD_M and alt_diff <= ALTITUDE_REACH_THRESHOLD_M:
                        print(f"Reached waypoint {i + 1} (Circle {circle + 1}).")
                        break
                time.sleep(0.1) # Small delay to avoid busy-waiting, but still poll frequently

        if circle==0:
            survey_rectangular_field()
            object_waypoints=read_waypoints_from_file("objects.txt")
            dropPayload(object_waypoints[0],9)

        if circle==1:
            dropPayload(object_waypoints[1],10)

        if circle==2:
            dropPayload(object_waypoints[2],11)

        if circle==3:
            dropPayload(object_waypoints[3],12)

    print("\n--- Mission complete! ---")
    # --- Land the drone ---
    print("Sending LAND command...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, # confirmation
        0, 0, 0, 0, 0, 0, 0
    )
    print("Drone should now be landing.")
    # Wait for the drone to land (check altitude near 0) - removed timeout
    while True:
        msg = master.recv_match(type=['GLOBAL_POSITION_INT'], blocking=True, timeout=1)
        if msg:
            current_alt_rel = msg.relative_alt / 1000.0
            print(f"Current relative altitude: {current_alt_rel:.2f}m (Landing...)")
            if current_alt_rel < ALTITUDE_REACH_THRESHOLD_M:
                print("Drone has landed.")
                break
        time.sleep(0.5)

    # --- Disarm the drone ---
    print("Disarming drone...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # confirmation
        0,  # disarm
        0, 0, 0, 0, 0, 0
    )
    print("Drone disarmed.")

    master.close()
    print("Connection closed.")

if __name__ == "__main__":
    run_drone_mission()