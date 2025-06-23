import time
import math
from pymavlink import mavutil

# --- Constants for waypoint navigation ---
WAYPOINT_REACH_THRESHOLD_M = 3.0  # Horizontal distance in meters to consider waypoint reached
ALTITUDE_REACH_THRESHOLD_M = 1.0  # Vertical distance in meters to consider target altitude reached
MISSION_TIMEOUT_SECONDS = 1800 # Max time for a full mission to prevent infinite loops
WAYPOINT_TIMEOUT_SECONDS = 60 # Max time for a single waypoint to be reached

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

def run_drone_mission():
    """
    Sets up and executes a drone mission with hardcoded waypoints and connection,
    including heartbeat, arm, takeoff, and real-time waypoint navigation.
    """

    # --- Configuration: Hardcoded Waypoints and Connection String ---
    # You can change these values directly in the script.
    # For Maryland area: Lat ~38.3148, Lon ~-76.5512
    waypoints = [
        {'lat': 38.3148446, 'lon': -76.5512221, 'alt': 20.0}, # Waypoint 1
        {'lat': 38.3155000, 'lon': -76.5515000, 'alt': 25.0}, # Waypoint 2
        {'lat': 38.3150000, 'lon': -76.5520000, 'alt': 30.0}, # Waypoint 3
        {'lat': 38.3145000, 'lon': -76.5510000, 'alt': 20.0}  # Waypoint 4
    ]
    # Adjust number of waypoints based on the list above
    num_waypoints = len(waypoints)

    # Common SITL connection string
    connection_string = 'udp:127.0.0.1:14550'

    # --- Display configured waypoints ---
    print("\n--- Waypoints configured ---")
    for i, wp in enumerate(waypoints):
        print(f"Waypoint {i+1}: Lat={wp['lat']:.6f}, Lon={wp['lon']:.6f}, Alt={wp['alt']:.2f}m")
    print(f"Connection String: {connection_string}")


    # --- Establish connection to the drone ---
    try:
        master = mavutil.mavlink_connection(connection_string, baud=57600)
        master.wait_heartbeat()
        print(f"Heartbeat from system (sys {master.target_system} comp {master.target_component})")
    except Exception as e:
        print(f"Error connecting to drone: {e}")
        print("Please ensure your drone is connected and the connection string is correct.")
        return

    print("Received initial heartbeat.")

    # --- Set mode to GUIDED ---
    print("Setting drone mode to GUIDED...")
    mode_id = master.mode_mapping()['GUIDED']
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    # Wait for mode to change
    start_time_mode_change = time.time()
    while True:
        msg = master.recv_match(type=['HEARTBEAT'], blocking=True, timeout=1)
        if msg:
            current_mode = mavutil.mode_string_v10(msg)
            if current_mode == "GUIDED":
                print("Drone is in GUIDED mode.")
                break
            else:
                print(f"Current mode: {current_mode}. Waiting for GUIDED mode...")
        elif time.time() - start_time_mode_change > 10:
            print("Timeout waiting for GUIDED mode. Please check drone status.")
            return
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

    # Wait until the drone reaches takeoff altitude
    print(f"Waiting for drone to reach {TAKEOFF_ALTITUDE}m altitude...")
    start_time_takeoff = time.time()
    while True:
        msg = master.recv_match(type=['GLOBAL_POSITION_INT'], blocking=True, timeout=1)
        if msg:
            current_alt_rel = msg.relative_alt / 1000.0 # GLOBAL_POSITION_INT.relative_alt is in mm
            print(f"Current relative altitude: {current_alt_rel:.2f}m / {TAKEOFF_ALTITUDE:.2f}m")
            if current_alt_rel >= TAKEOFF_ALTITUDE - ALTITUDE_REACH_THRESHOLD_M:
                print(f"Drone reached target takeoff altitude of {current_alt_rel:.2f}m.")
                break
        if time.time() - start_time_takeoff > WAYPOINT_TIMEOUT_SECONDS:
            print("Takeoff timed out. Please check drone status, GPS lock, and pre-arm checks.")
            return
        time.sleep(0.5)


    # --- 7. Iterate through all the waypoints, making sure the drone flies through each and every point at the specified altitude. ---
    # --- 8. The waypoints must be covered 4 times. ---
    NUM_CIRCLES = 4

    # Type mask for SET_POSITION_TARGET_GLOBAL_INT to only use position (ignore velocity, accel, yaw)

    mission_start_time = time.time()

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
                0b110000000000, # Only use position, ignore velocity, accel, yaw
                int(wp['lat'] * 1e7), # lat_int (degrees * 10^7)
                int(wp['lon'] * 1e7), # lon_int (degrees * 10^7)
                wp['alt'], # alt (meters)
                0, 0, 0, # vx, vy, vz (ignored by mask)
                0, 0, 0, # afx, afy, afz (ignored by mask)
                0, 0 # yaw, yaw_rate (ignored by mask)
            )

            # Wait for the drone to reach the waypoint by checking its GPS position
            waypoint_start_time = time.time()
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
                        break # Exit inner while loop, move to next waypoint

                if time.time() - waypoint_start_time > WAYPOINT_TIMEOUT_SECONDS:
                    print(f"Timeout: Drone failed to reach waypoint {i + 1} within {WAYPOINT_TIMEOUT_SECONDS} seconds.")
                    print("Aborting mission.")
                    return

                if time.time() - mission_start_time > MISSION_TIMEOUT_SECONDS:
                    print(f"Mission timeout: Entire mission exceeded {MISSION_TIMEOUT_SECONDS} seconds.")
                    print("Aborting mission.")
                    return

                time.sleep(0.1) # Small delay to avoid busy-waiting, but still poll frequently

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
    start_time_landing = time.time()
    while True:
        msg = master.recv_match(type=['GLOBAL_POSITION_INT'], blocking=True, timeout=1)
        if msg:
            current_alt_rel = msg.relative_alt / 1000.0
            print(f"Current relative altitude: {current_alt_rel:.2f}m (Landing...)")
            if current_alt_rel < ALTITUDE_REACH_THRESHOLD_M: # Close to ground
                print("Drone has landed.")
                break
        if time.time() - start_time_landing > WAYPOINT_TIMEOUT_SECONDS:
            print("Landing timed out. Please check drone status.")
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
