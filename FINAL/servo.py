from pymavlink import mavutil
import time

# --- 1. Establish the connection ---
# Choose ONE of these connection types that matches your setup:
# -----------------------------------------------------------
# To MAVProxy (recommended if you're using MAVProxy for monitoring):
master = mavutil.mavlink_connection('udp:127.0.0.1:14550') # Or your MAVProxy's output UDP port

# Direct serial connection (replace with your port and baud rate):
# master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)

# TCP connection (e.g., to SITL or a network bridge):
# master = mavutil.mavlink_connection('tcp:127.0.0.1:5760')

print("Connecting...")

# --- 2. Wait for heartbeat to ensure connection and set target_system/component ---
master.wait_heartbeat()
print(f"Heartbeat from system (system {master.target_system} component {master.target_component})")

# --- 3. Send the command ---
print("Sending MAV_CMD_DO_SET_SERVO command...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
    0,      # confirmation
    10,      # param1: servo instance number (channel 9)
    2000,   # param2: PWM value
    0, 0, 0, 0, 0 # unused params
)
# print("Command sent.")
# --- 4. Listen for COMMAND_ACK (Crucial for debugging!) ---
# This tells you if the autopilot received and processed the command.
# ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5) # Wait up to 5 seconds
# if ack_msg:
#     print(f"COMMAND_ACK received: Result: {ack_msg.result}")
#     if ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
#         print("Command was accepted by the autopilot!")
#     elif ack_msg.result == mavutil.mavlink.MAV_RESULT_DENIED:
#         print("Command was DENIED by the autopilot (check mode, arming, or function parameter).")
#     else:
#         print(f"Command result was {ack_msg.result} (check MAV_RESULT enum for details).")
# else:
#     print("No COMMAND_ACK received. Command might not have reached the autopilot or timed out.")
time.sleep(20)
for i in range(9,13):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,      # confirmation
        i,      # param1: servo instance number (channel 9)
        0,   # param2: PWM value
        0, 0, 0, 0, 0 # unused params
    )
print("Command sent.")

# --- 5. Keep the script alive briefly to allow command processing ---
time.sleep(2) # Give the autopilot a moment to act
print("Script finished.")