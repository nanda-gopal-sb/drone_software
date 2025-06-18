from pymavlink import mavutil
import time

# Set up the connection to the MAVLink vehicle
# Replace with your connection string (e.g., 'udp:127.0.0.1:14550' for a local simulation or 'COMx' for a serial connection)
connection_string = 'udp:127.0.0.1:14550'  # Change to your connection string
master = mavutil.mavlink_connection(connection_string)

# Wait for the heartbeat message to ensure that the connection is established
master.wait_heartbeat()
print("Heartbeat from system (ID: %d)" % master.target_system)

# Set mode to GUIDED
mode = 'GUIDED'
mode_id = master.mode_mapping()[mode]
master.set_mode(mode_id)
print(f"Mode set to {mode}")

# Wait for the mode change to take effect
time.sleep(1)

# Define the waypoints (latitude, longitude, altitude) in degrees and meters
waypoints = [
    (47.397742, -122.073115, 10),  # WP1: Latitude, Longitude, Altitude
    (47.397743, -122.073116, 10),  # WP2
    (47.397744, -122.073117, 10),  # WP3
    (47.397745, -122.073118, 10),  # WP4
]

# Create a mission (waypoints) and send them to the vehicle
# First, we need to clear any existing missions
master.mav.mission_clear_all_send(master.target_system, master.target_component)

# Send each waypoint
for i, (lat, lon, alt) in enumerate(waypoints):
    master.mav.mission_item_send(
        master.target_system,
        master.target_component,
        i,  # Sequence number for the waypoint
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # Coordinate frame (use relative altitude)
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  # Command type for waypoint
        0,  # Current waypoint (0 = false, 1 = true)
        0,  # Autocontinue (1 = Yes, 0 = No)
        0,  # Param 1: hold time in decimal seconds (not used here)
        0,  # Param 2: acceptance radius (not used here)
        0,  # Param 3: pass radius (not used here)
        0,  # Param 4: yaw angle (not used here)
        lat,  # Latitude
        lon,  # Longitude
        alt   # Altitude
    )
    print(f"Waypoint {i+1} sent: Latitude {lat}, Longitude {lon}, Altitude {alt} meters")

# Command the vehicle to start the mission (start flying to the waypoints)
master.mav.mission_start_send(master.target_system, master.target_component, 0)
print("Mission started. The vehicle will start navigating the waypoints.")

# Monitor the vehicle's mission status (optional)
while True:
    msg = master.recv_match(type='MISSION_ITEM_REACHED', blocking=True)
    print(f"Reached waypoint {msg.seq}")
    time.sleep(1)
