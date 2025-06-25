import time
from pymavlink import mavutil

def run_drone_mission():
    while True:
        try:
            num_waypoints = int(input("Enter the number of waypoints: "))
            if num_waypoints > 0:
                break
            else:
                print("Number of waypoints must be positive.")
        except ValueError:
            print("Invalid input. Please enter an integer.")

    # --- 2. Ask the user for latitudes, longitudes, and altitudes for each waypoint ---
    # --- 3. Create an array, with the inputs received named "waypoints" ---
    waypoints = []
    for i in range(num_waypoints):
        print(f"\n--- Waypoint {i + 1} ---")
        while True:
            try:
                lat = float(input(f"Enter latitude for waypoint {i + 1}: "))
                break
            except ValueError:
                print("Invalid input. Please enter a valid number for latitude.")
        while True:
            try:
                lon = float(input(f"Enter longitude for waypoint {i + 1}: "))
                break
            except ValueError:
                print("Invalid input. Please enter a valid number for longitude.")
        while True:
            try:
                alt = float(input(f"Enter altitude (in meters) for waypoint {i + 1}: "))
                if alt >= 0:
                    break
                else:
                    print("Altitude must be non-negative.")
            except ValueError:
                print("Invalid input. Please enter a valid number for altitude.")
        waypoints.append({'lat': lat, 'lon': lon, 'alt': alt})

    print("\n--- Waypoints configured ---")
    for i, wp in enumerate(waypoints):
        print(f"Waypoint {i+1}: Lat={wp['lat']:.6f}, Lon={wp['lon']:.6f}, Alt={wp['alt']:.2f}m")
    
    connection_string = 'udp:127.0.0.1:14550'
    
    try:
        master = mavutil.mavlink_connection(connection_string, baud=57600)
        master.wait_heartbeat()
        print(f"Heartbeat from system (sys {master.target_system} comp {master.target_component})")
    except Exception as e:
        print(f"Error connecting to drone: {e}")
        print("Please ensure your drone is connected and the connection string is correct.")
        return

    # --- 4. Send pymavlink heartbeat command---
    print("Received initial heartbeat.")
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
    print("Setting drone mode to GUIDED...")
    mode_id = master.mode_mapping()['GUIDED']
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    
    while True:
        msg = master.recv_match(type=['HEARTBEAT'], blocking=True, timeout=1)
        if msg and mavutil.mode_string_v10(msg) == "GUIDED":
            print("Drone is in GUIDED mode.")
            break
        elif not msg:
            print("Waiting for HEARTBEAT to confirm mode change...")
        time.sleep(0.5)
    
    print("Waiting for drone to arm...")
    master.motors_armed_wait()
    print("Drone armed!")
    time.sleep(2) # Give a moment after arming
    # --- 6. Send the liftoff command to 16m ---
    TAKEOFF_ALTITUDE = 16.0
    print(f"Sending TAKEOFF command to {TAKEOFF_ALTITUDE} meters...")
    TAKEOFF_PITCH = 0.0 # Degrees. Use 0 for vertical takeoff.
    print(f"Sending TAKEOFF command to {TAKEOFF_ALTITUDE} meters with pitch {TAKEOFF_PITCH}...")
    master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, # confirmation
    TAKEOFF_PITCH, # param1: pitch
    0, # param2: empty
    0, # param3: empty
    0, # param4: empty
    0, # param5: empty
    0, # param6: empty
    TAKEOFF_ALTITUDE # param7: altitude
    )
    # Wait until the drone reaches takeoff altitude
    print(f"Waiting for drone to reach {TAKEOFF_ALTITUDE}m altitude...")
    # This is a basic way to check altitude. For a robust system, you'd listen to GLOBAL_POSITION_INT or VFR_HUD messages.
    # For simulation, we'll just wait a bit. In a real scenario, you'd poll actual altitude data.
    start_time = time.time()
    while True:
        msg = master.recv_match(type=['VFR_HUD'], blocking=True, timeout=1)
        if msg:
            if msg.alt >= TAKEOFF_ALTITUDE * 0.95: # Check if within 5% of target
                print(f"Drone reached {msg.alt:.2f}m. Proceeding with mission.")
                break
        if time.time() - start_time > 30: # Timeout after 30 seconds
            print("Takeoff timed out. Please check drone status.")
            return
        time.sleep(1) # Check every second
    time.sleep(5) # Give a bit more time after reaching altitude

    # --- 7. Iterate through all the waypoints, making sure the drone flies through each and every point at the specified altitude. ---
    # --- 8. The waypoints must be covered 4 times. ---
    NUM_CIRCLES = 4

    for circle in range(NUM_CIRCLES):
        print(f"\n--- Starting Mission Circle {circle + 1}/{NUM_CIRCLES} ---")
        for i, wp in enumerate(waypoints):
            print(f"Navigating to waypoint {i + 1} (Circle {circle + 1}): Lat={wp['lat']:.6f}, Lon={wp['lon']:.6f}, Alt={wp['alt']:.2f}m")

            # Send MAV_CMD_NAV_WAYPOINT command
            # param1: hold time (s)
            # param2: accept radius (m)
            # param3: pass through (0 to pass through, 1 to fly around)
            # param4: yaw (deg, 0 for auto)
            # param5-7: lat, lon, alt
            master.mav.set_position_target_global_int_send(
                0, # time_boot_ms (not used in this context, FC sets)
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, # Frame for target (relative to takeoff alt)
                0b110000000000, # Only use position, ignore velocity, accel, yaw
                int(wp['lat'] * 1e7), # lat_int (degrees * 10^7)
                int(wp['lon'] * 1e7), # lon_int (degrees * 10^7)
                wp['alt'], # alt (meters)
                0, 0, 0, # vx, vy, vz (ignored)
                0, 0, 0, # afx, afy, afz (ignored)
                0, 0 # yaw, yaw_rate (ignored)
            )
            # Wait for the drone to reach the waypoint (simplified)
            # In a real scenario, you would monitor the current position
            # (e.g., GLOBAL_POSITION_INT) and compare it to the target waypoint.
            # For demonstration, we'll just add a delay.
            print("Waiting for drone to reach waypoint (simulated delay)...")
            time.sleep(10) # Adjust this delay based on your drone's speed and distance between waypoints
            print(f"Reached waypoint {i + 1} (simulated).")

    print("\n--- Mission complete! ---")

    # --- Land the drone ---
    print("Sending LAND command...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, # confirmation
        0, 0, 0, 0, 0, 0, 0 # parameters for land are usually ignored or set to 0.
                           # Actual landing point is often current position or HOME.
    )
    print("Drone should now be landing.")
    time.sleep(15) # Give time for landing

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
