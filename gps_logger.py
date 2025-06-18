import csv
import time
from pymavlink import mavutil# Change the connection string to match your setup (localhost, port 14550)

connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')

# Open the CSV file in write mode, write header first if it doesn't exist
csv_filename = 'gps_log.csv'
header = ['Timestamp', 'Latitude', 'Longitude', 'Altitude']

# Check if file exists and create the file with headers if not
try:
    with open(csv_filename, 'r') as f:
        pass
except FileNotFoundError:
    with open(csv_filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(header)

# Function to log GPS data
def log_gps_data():
    while True:
        # Wait for GPS data (GPGGA message)
        msg = connection.recv_match(type='GPS_RAW_INT', blocking=True)
        
        if msg:
            latitude = msg.lat / 1E7  # Convert to degrees
            longitude = msg.lon / 1E7  # Convert to degrees
            altitude = msg.alt / 1000.0  # Convert to meters
            timestamp = time.strftime('%Y-%m-%d %H:%M:%S', time.gmtime())
            with open(csv_filename, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([timestamp, latitude, longitude, altitude])
            print(f"{timestamp} - Latitude: {latitude}, Longitude: {longitude}, Altitude: {altitude} meters")
        
        time.sleep(1)  # Log data every second

if __name__ == '__main__':
    print("Starting GPS logger...")
    log_gps_data()
