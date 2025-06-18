from pymavlink import mavutil
import time

connection_string = 'udp:127.0.0.1:14550' 
master = mavutil.mavlink_connection(connection_string)

master.wait_heartbeat()
print("Heartbeat from system (ID: %d)" % master.target_system)

mode = 'GUIDED'
mode_id = master.mode_mapping()[mode]
master.set_mode(mode_id)
print(f"Mode set to {mode}")

time.sleep(1)

waypoints = [
    (10.050586, 76.330783, 15),
    (10.047826, 76.329881, 15),  
    (10.047053, 76.331717, 15),  
    (10.049185, 76.332703, 15),  
]
master.mav.mission_clear_all_send(master.target_system, master.target_component)

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

master.mav.mission_ack_send(master.target_system, master.target_component, 0)
print("Mission started. The vehicle will start navigating the waypoints.")

while True:
    msg = master.recv_match(type='MISSION_ITEM_REACHED', blocking=True)
    print(f"Reached waypoint {msg.seq}")
    time.sleep(1)
