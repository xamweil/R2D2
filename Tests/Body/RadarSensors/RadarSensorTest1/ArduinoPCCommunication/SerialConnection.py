import serial
import time
import struct
import json
import os

# Configure the serial port and baud rate
SERIAL_PORT = 'COM6'  # Replace with your actual COM port
BAUD_RATE = 256000
OUTPUT_FILE = 'sensor_data.json'

def parse_data(data):
    try:
        if data.startswith(b'\xF4\xF3\xF2\xF1') and data.endswith(b'\xF8\xF7\xF6\xF5'):
            length = struct.unpack('<H', data[4:6])[0]
            data_type = data[6]
            header = data[7]
            if data_type == 0x02 and header == 0xAA:
                target_status = data[8]
                movement_distance = struct.unpack('<H', data[9:11])[0]
                movement_energy = data[11]
                stationary_distance = struct.unpack('<H', data[12:14])[0]
                stationary_energy = data[14]
                detection_distance = struct.unpack('<H', data[15:17])[0]

                parsed_data = {
                    "target_status": target_status,
                    "movement_distance": movement_distance,
                    "movement_energy": movement_energy,
                    "stationary_distance": stationary_distance,
                    "stationary_energy": stationary_energy,
                    "detection_distance": detection_distance
                }

                print(parsed_data)
                write_data_to_file(parsed_data)
            else:
                print("Unknown data type or header.")
        else:
            print("Invalid frame format.")
    except Exception as e:
        print(f"Error parsing data: {e}")

def write_data_to_file(data):
    try:
        # Append data to the file in real-time
        with open(OUTPUT_FILE, 'a') as file:
            file.write(json.dumps(data) + '\n')
    except Exception as e:
        print(f"Error writing data to file: {e}")

def read_sensor_data():
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
            time.sleep(2)  # Allow time for connection setup

            # Clear the output file at the start
            if os.path.exists(OUTPUT_FILE):
                os.remove(OUTPUT_FILE)

            while True:
                if ser.in_waiting > 0:
                    data = ser.read(ser.in_waiting)
                    parse_data(data)
                else:
                    time.sleep(1)  # Wait before checking again
    except serial.SerialException as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    read_sensor_data()
