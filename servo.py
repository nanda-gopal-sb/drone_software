# pymavlink_servo_control.py

import time
from pymavlink import mavutil

# --- Configuration ---
# Replace with your Pixhawk connection details.
# Common options:
# Serial: '/dev/ttyACM0' (Linux), 'COM3' (Windows)
# UDP: 'udp:0.0.0.0:14550' (listen for incoming connection)
# UDP: 'udp:127.0.0.1:14550' (send to a specific IP/port, e.g., SITL)
# For a direct USB connection, it's usually a serial port.
connection_string = '/dev/ttyACM0' # Example for SITL or a UDP bridge
# connection_string = '/dev/ttyACM0' # Example for Linux USB connection
# connection_string = 'COM3' # Example for Windows USB connection

# Baudrate for serial connections. Ignored for UDP.
baud_rate = 57600

# Pixhawk servo output channel number (1-based index).
# For example, if connected to AUX1, this would be 9 (for ArduCopter/Plane AUX1 is output channel 9).
# Consult your flight controller's documentation for the correct mapping.
# For PX4, AUX1 is often mapped to channel 5 or 6.
servo_channel_number = 9 # Example: ArduPilot AUX1 (physical output channel 9)

# Desired PWM value for the servo (in microseconds).
# Typical range is 1000 (min) to 2000 (max), with 1500 as center.
target_pwm_value = 1700 # Example: Move servo to a specific position

# Duration to hold the servo in the target position (seconds)
hold_duration = 3

def send_set_servo_command(master, servo_id, pwm_value):
    """
    Sends a MAV_CMD_DO_SET_SERVO command to the Pixhawk.

    Args:
        master: The MAVLink connection object.
        servo_id: The 1-based servo output channel number (e.g., 1-14).
                  This maps to the physical output pins configured on the Pixhawk.
        pwm_value: The desired PWM value in microseconds (e.g., 1000-2000).
    """
    print(f"Sending MAV_CMD_DO_SET_SERVO: Servo ID {servo_id}, PWM {pwm_value} us")
    master.mav.command_long_send(
        master.target_system,       # Target system ID (usually 1 for Pixhawk)
        master.target_component,    # Target component ID (usually 1 for Pixhawk)
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO, # Command ID
        0,                          # Confirmation (0 for no confirmation)
        servo_id,                   # Param1: Servo number
        pwm_value,                  # Param2: PWM value (in microseconds)
        0, 0, 0, 0, 0               # Unused parameters
    )
    print("Command sent. Waiting for ACK...")
    ack_received = False
    start_time = time.time()
    while not ack_received and (time.time() - start_time < 5): # Wait for 5 seconds for ACK
        msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=0.1)
        if msg:
            if msg.command == mavutil.mavlink.MAV_CMD_DO_SET_SERVO:
                if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    print("COMMAND_ACK: Accepted!")
                    ack_received = True
                else:
                    print(f"COMMAND_ACK: Failed with result {msg.result}")
                    ack_received = True
        time.sleep(0.01) # Small delay to avoid busy-waiting

    if not ack_received:
        print("No COMMAND_ACK received within timeout.")


def main():
    """
    Main function to connect to Pixhawk and control a servo.
    """
    print(f"Connecting to Pixhawk via {connection_string}...")

    # Create a MAVLink connection
    master = mavutil.mavlink_connection(connection_string, baud=baud_rate) # For serial
    #master = mavutil.mavlink_connection(connection_string) # For UDP

    # Wait for the first heartbeat message to ensure connection is established
    # This populates master.target_system and master.target_component
    print("Waiting for Pixhawk heartbeat...")
    master.wait_heartbeat()
    print(f"Heartbeat from system (SYSID {master.target_system}, COMPID {master.target_component})")

    print("\n--- Sending Servo Control Command ---")
    send_set_servo_command(master, servo_channel_number, target_pwm_value)

    print(f"Servo set to {target_pwm_value} us. Holding for {hold_duration} seconds...")
    time.sleep(hold_duration)

    # Optionally, set the servo back to a neutral position
    print("Setting servo back to 1500 us (neutral)...")
    send_set_servo_command(master, servo_channel_number, 1500)
    time.sleep(2) # Give it time to move

    print("\nScript finished.")


if __name__ == "__main__":
    main()
